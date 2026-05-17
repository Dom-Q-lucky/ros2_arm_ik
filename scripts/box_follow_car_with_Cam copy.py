import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
import cv2
import numpy as np
import ikpy.chain
import os
import math
import time

class FullPandaTaskCommander(Node):
    def __init__(self):
        super().__init__('full_panda_task_commander', 
                         parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        self.arm_pub = self.create_publisher(JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        self.hand_pub = self.create_publisher(JointTrajectory, '/panda_hand_controller/joint_trajectory', 10)
        self.cmd_pub = self.create_publisher(Twist, '/model/mobile_panda/cmd_vel', 10)
        
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.base_image_sub = self.create_subscription(Image, '/base_camera/image_raw', self.base_image_callback, 10)
        self.bridge = CvBridge()

        urdf_path = os.path.expanduser('~/panda_ws/src/panda_description/urdf/panda_fixed.urdf')
        self.chain = ikpy.chain.Chain.from_urdf_file(urdf_path, base_elements=["world"])
        for i, link in enumerate(self.chain.links):
            link.is_kinematic = True if 2 <= i <= 8 else False

        self.state = "INITIALIZE" 
        self.init_counter = 0
        
        # 维持低头巡航姿态
        self.current_xyz = [0.4, 0.0, 0.15] 
        self.target_found = False
        self.err_u, self.err_v = 0, 0
        self.lost_frames_counter = 0
        self.align_stable_frames = 0
        
        self.k_p = 0.00012          
        self.deadzone = 25
        
        self.base_target_found = False
        self.base_err_u = 0
        self.base_target_area = 0

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('--- 修复版：小车与机械臂视觉算法均已全量加载 ---')

    # ==========================================
    # 小车视觉算法（已恢复滤波与可视化窗口！）
    # ==========================================
    def base_image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([100, 150, 80]), np.array([130, 255, 255]))
        
        # 补回：形态学去噪，避免小车乱跑
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > 300:
                M = cv2.moments(c)
                if M["m00"] != 0:
                    self.base_target_found = True
                    self.base_err_u = int(M["m10"] / M["m00"]) - 320
                    self.base_target_area = area
                    
                    # 补回：OSD数据显示与准星，方便你调参数
                    cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                    cv2.circle(frame, (cx, cy), 8, (255, 0, 0), -1)
                    cv2.putText(frame, f"Nav Area: {int(area)}", (10, 40), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                    cv2.putText(frame, f"State: {self.state}", (10, 80), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            self.base_target_found = False

        # 补回：必须展示出来
        cv2.imshow("Base Nav View", frame)
      #  cv2.imshow("Base Nav Mask", mask)
        cv2.waitKey(1)

    # ==========================================
    # 机械臂视觉算法（绝对不动你的）
    # ==========================================
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([100, 150, 80]), np.array([130, 255, 255]))
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 200:
                M = cv2.moments(c)
                if M["m00"] != 0:
                    self.err_u = int(M["m10"] / M["m00"]) - 320
                    self.err_v = int(M["m01"] / M["m00"]) - 240
                    self.target_found = True
                    self.lost_frames_counter = 0
                    cv2.circle(frame, (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])), 8, (0, 0, 255), -1)
        else:
            self.lost_frames_counter += 1
            if self.lost_frames_counter > 30: 
                self.target_found = False
            
        cv2.imshow("Arm View (Grasp)", frame)
       # cv2.imshow("Arm Mask (Grasp)", mask)
        cv2.waitKey(1)

    # ==========================================
    # 状态机：解耦逻辑（车停死，手才动）
    # ==========================================
    def control_loop(self):
        cmd = Twist()
        
        if self.state == "INITIALIZE":
            self.send_arm_pose(self.current_xyz, 2.0)
            self.init_counter += 1
            if self.init_counter > 40: 
                self.state = "BASE_SEARCH"
            return

        elif self.state == "BASE_SEARCH":
            if self.base_target_found:
                self.state = "BASE_APPROACH"
            else:
                cmd.angular.z = 0.5
                self.cmd_pub.publish(cmd)

        elif self.state == "BASE_APPROACH":
            if self.base_target_found:
                cmd.linear.x = 0.25
                cmd.angular.z = -float(self.base_err_u) * 0.006
                
                if self.base_target_area > 28000:
                    self.cmd_pub.publish(Twist()) # 小车强制刹车
                    self.lost_frames_counter = 0  # 重置机械臂的丢帧计数，防止乱跳
                    self.state = "ALIGN"
                else:
                    self.cmd_pub.publish(cmd)
            else:
                self.state = "BASE_SEARCH"

        elif self.state == "ALIGN":
            self.cmd_pub.publish(Twist()) # 全程锁死小车轮子
            
            if not self.target_found:
                self.send_arm_pose(self.current_xyz, 0.1)
                if self.lost_frames_counter > 40:
                    self.state = "BASE_SEARCH" 
                return

            dx = self.err_v * self.k_p
            dy = self.err_u * self.k_p
            alpha = 0.05
            
            raw_target_x = self.current_xyz[0] + dx
            raw_target_y = self.current_xyz[1] + dy

            self.current_xyz[0] = (1 - alpha) * self.current_xyz[0] + alpha * raw_target_x
            self.current_xyz[1] = (1 - alpha) * self.current_xyz[1] + alpha * raw_target_y
            self.current_xyz[2] = 0.15 
            
            error_magnitude = math.sqrt(self.err_u**2 + self.err_v**2)
            if error_magnitude < 8:
                self.send_arm_pose(self.current_xyz, 0.4)
            else:
                self.send_arm_pose(self.current_xyz, 0.2)

            if abs(self.err_u) < self.deadzone and abs(self.err_v) < self.deadzone:
                self.align_stable_frames += 1
                if self.align_stable_frames > 10:
                    self.execute_grasp_sequence() 
                    self.state = "DONE"
            else:
                self.align_stable_frames = 0

    def send_arm_pose(self, target_xyz, duration_sec):
        warm_start = [0.0] * 12
        warm_start[3], warm_start[5], warm_start[7] = -0.785, -2.356, 1.571
        try:
            joint_angles = self.chain.inverse_kinematics(target_xyz, 
                                                         target_orientation=[0, 0, -1], 
                                                         orientation_mode='Z', 
                                                         initial_position=warm_start)
            msg = JointTrajectory()
            msg.joint_names = [f'panda_joint{i}' for i in range(1, 8)]
            point = JointTrajectoryPoint()
            point.positions = list(joint_angles[2:9])
            point.time_from_start = Duration(sec=0, nanosec=int(duration_sec * 1e9))
            msg.points = [point]
            self.arm_pub.publish(msg)
        except: pass

    def set_gripper(self, distance, duration_sec=1):
        msg = JointTrajectory()
        msg.joint_names = ['panda_finger_joint1', 'panda_finger_joint2']
        point = JointTrajectoryPoint()
        point.positions = [distance, distance]
        point.time_from_start = Duration(sec=duration_sec, nanosec=0)
        msg.points = [point]
        self.hand_pub.publish(msg)

    # ==========================================
    # 你的原生动作序列
    # ==========================================
    def execute_grasp_sequence(self):
        self.get_logger().info('--- 开始最终抓取序列 ---')
        
        grasp_offset_x = 0.0  
        grasp_offset_y = 0.00  
        grasp_offset_z = -0.07  

        self.set_gripper(0.05)
        time.sleep(1.0)
        
        final_x = self.current_xyz[0] + grasp_offset_x
        final_y = self.current_xyz[1] + grasp_offset_y
        

        self.send_arm_pose([final_x, final_y, grasp_offset_z], 1.5)
        time.sleep(2.0)

        self.set_gripper(0.01) 
        time.sleep(3.0) 

        self.send_arm_pose([final_x, final_y, 0.3], 2.0)
        time.sleep(3.0)

        platform_pos = [-0.6, -0.4, 0.2]
        self.send_arm_pose(platform_pos, 4.0)
        time.sleep(5.0)
        self.set_gripper(0.04)
        self.get_logger().info('任务圆满完成！')

def main():
    rclpy.init()
    node = FullPandaTaskCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()