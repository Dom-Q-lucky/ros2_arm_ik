import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
import cv2
import numpy as np
import ikpy.chain
import os
import math
import time

class PandaVisionCommander(Node):
    def __init__(self):
        super().__init__('panda_vision_commander', 
                         parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # 1. 基础设置
        self.arm_pub = self.create_publisher(JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        self.hand_pub = self.create_publisher(JointTrajectory, '/panda_hand_controller/joint_trajectory', 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        # 2. 加载 IK 链
        urdf_path = os.path.expanduser('~/panda_ws/src/panda_description/urdf/panda_fixed.urdf')
        self.chain = ikpy.chain.Chain.from_urdf_file(urdf_path, base_elements=["world"])
        for i, link in enumerate(self.chain.links):
            link.is_kinematic = True if 2 <= i <= 8 else False

        # 3. 状态机变量
        self.state = "SEARCH" 
        self.target_found = False
        self.current_xyz = [0.4, 0.0, 0.45]
        self.err_u, self.err_v = 0, 0
        
        # 计时器替代方案 (1.0s = 20次 0.05s 的循环)
        self.wait_counter = 0
        self.lost_frames_counter = 0
        self.align_stable_frames = 0
        
        # --- 核心参数调节 ---
        self.search_angle = 0.0
        self.k_p = 0.00012          # 再次降低，慢速对准
        self.deadzone = 25
        self.search_speed = 0.01    # 极慢搜索
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('--- 鲁棒视觉系统：非阻塞模式启动 ---')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([100, 150, 180]), np.array([130, 255, 255]))
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        cv2.imshow("Binary Mask (Black & White)", mask)
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
            if self.lost_frames_counter > 30: # 1.5秒宽限
                self.target_found = False
            
        cv2.imshow("Panda Debug", frame)
        cv2.waitKey(1)

    def control_loop(self):
        if self.state == "SEARCH":
            if not self.target_found:
                # 环绕搜索
                self.search_angle += self.search_speed
                self.current_xyz[0] = 0.45 + 0.15 * math.cos(self.search_angle)
                self.current_xyz[1] = 0.0 + 0.15 * math.sin(self.search_angle)
                self.current_xyz[2] = 0.45
                self.send_arm_pose(self.current_xyz, 0.1)
            else:
                self.get_logger().info('看到方块！进入刹车状态...')
                self.state = "BRAKE"
                self.wait_counter = 20 # 刹车 1 秒

        elif self.state == "BRAKE":
            # 原地定格，等画面稳下来
            self.send_arm_pose(self.current_xyz, 0.1)
            self.wait_counter -= 1
            if self.wait_counter <= 0:
                self.state = "ALIGN"
                self.get_logger().info('开始微调对齐...')

        elif self.state == "ALIGN":
            if not self.target_found:
                self.get_logger().warn('丢失目标，原地等待重捕...')
                self.send_arm_pose(self.current_xyz, 0.1)
                if self.lost_frames_counter > 40:
                    self.state = "SEARCH" # 彻底丢了才回去转圈
                return

            # --- 映射逻辑检测区 ---
            # 如果机器人“躲着”方块走，请把下面的减号改成加号！
            dx = self.err_v * self.k_p
            dy = self.err_u * self.k_p

            alpha = 0.05
            # 极小步进限制
            raw_target_x = self.current_xyz[0] + dx
            raw_target_y = self.current_xyz[1] + dy

            self.current_xyz[0] = (1 - alpha) * self.current_xyz[0] + alpha * raw_target_x
            self.current_xyz[1] = (1 - alpha) * self.current_xyz[1] + alpha * raw_target_y
            self.current_xyz[2] = 0.35 
            
            error_magnitude = math.sqrt(self.err_u**2 + self.err_v**2)
            if error_magnitude < 8:
                self.send_arm_pose(self.current_xyz, 0.4)
            else:
                self.send_arm_pose(self.current_xyz, 0.2)

            if abs(self.err_u) < self.deadzone and abs(self.err_v) < self.deadzone:
                self.align_stable_frames += 1
                if self.align_stable_frames > 10:
                    self.get_logger().info('对准成功！执行抓取...')
                    self.execute_grasp_sequence() # 抓取逻辑包含 sleep 是可以的，因为它是单次执行
                    self.state = "DONE"
            else:
                self.align_stable_frames = 0

    def send_arm_pose(self, target_xyz, duration_sec):
        # 延续你 pick 代码里的 IK 设置
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
        except Exception as e:
            self.get_logger().error(f'IK 求解失败: {e}')

    # ... set_gripper 和 execute_grasp_sequence 保持原来的逻辑不变 ...

    def set_gripper(self, distance, duration_sec=1):
        msg = JointTrajectory()
        msg.joint_names = ['panda_finger_joint1', 'panda_finger_joint2']
        point = JointTrajectoryPoint()
        point.positions = [distance, distance]
        point.time_from_start = Duration(sec=duration_sec, nanosec=0)
        msg.points = [point]
        self.hand_pub.publish(msg)

    def execute_grasp_sequence(self):
            self.get_logger().info('--- 开始最终抓取序列 ---')
            
            # 1. 物理中心补偿 (根据视频观察调整)
            # 如果夹爪偏前了，就减小 x_offset；如果偏左了，就调整 y_offset
            # 这三个参数需要你根据刚才“差一点点”的方向手动微调
            grasp_offset_x = 0.008  # 示例：相机在夹爪前方 4.5 厘米
            grasp_offset_y = 0.020  # 示例：左右偏差
            grasp_offset_z = 0.025  # 抓取时的 Z 高度 (0.05~0.06 通常刚好)

            # 2. 张开爪子并保持对齐坐标
            self.set_gripper(0.04)
            time.sleep(1.0)
            
            # 3. 精准下降 (带补偿的坐标)
            final_x = self.current_xyz[0] + grasp_offset_x
            final_y = self.current_xyz[1] + grasp_offset_y
            
            self.get_logger().info(f'正在下降到：X={final_x:.3f}, Y={final_y:.3f}')
            
            # 第一步：先下降到接近高度
            self.send_arm_pose([final_x, final_y, 0.15], 2.0)
            time.sleep(2.0)
            
            # 第二步：扎下去（抓取深度）
            self.send_arm_pose([final_x, final_y, grasp_offset_z], 1.5)
            time.sleep(2.0)

            # 4. 实施抓取
            self.get_logger().info('闭合夹爪...')
            self.set_gripper(0.01) # 10cm 方块，闭合到 0.01 确保捏紧
            time.sleep(3.0) # 关键：给 Gazebo 物理引擎时间建立摩擦力

            # 5. 起吊测试
            self.get_logger().info('起吊检查...')
            self.send_arm_pose([final_x, final_y, 0.3], 2.0)
            time.sleep(3.0)

            # 6. 转移到平台
            platform_pos = [-0.6, -0.4, 0.2]
            self.send_arm_pose(platform_pos, 4.0)
            time.sleep(5.0)
            self.set_gripper(0.04)
            self.get_logger().info('任务圆满完成！')

def main():
    rclpy.init()
    node = PandaVisionCommander()
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