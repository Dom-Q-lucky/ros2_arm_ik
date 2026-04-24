import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import ikpy.chain
import os
import time

class PandaPickCommander(Node):
    def __init__(self):
        super().__init__('panda_pick_commander', 
                         parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # 1. 创建两个发布者：一个管胳膊，一个管手指
        self.arm_pub = self.create_publisher(JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        self.hand_pub = self.create_publisher(JointTrajectory, '/panda_hand_controller/joint_trajectory', 10)
      #  self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # 2. 加载 IK 链
        urdf_path = os.path.expanduser('~/panda_ws/src/panda_description/urdf/panda_fixed.urdf')
        self.chain = ikpy.chain.Chain.from_urdf_file(urdf_path, base_elements=["world"])
        for i, link in enumerate(self.chain.links):
            if 2 <= i <= 8:
                link.is_kinematic = True
            else:
                link.is_kinematic = False

        # 3. 核心坐标定义 [x, y, z]
        
        self.pos_cube_above = [0.5, 0.0, 0.12]    # 方块上方
        offect_x = 0.05
        offect_y = 0.001
        offect_z = -0.02
       
        self.pos_cube_grasp = [0.5 + offect_x, 0.0 + offect_y, 0.06 + offect_z]    # 抓取高度（手心刚好卡住方块中心）
        self.pos_platform_above = [-0.6, -0.4, 0.2] # 平台上方
        
        print([link.name for link in self.chain.links])
      
        self.get_logger().info('--- 抓取指挥官启动：目标蓝色方块，终点绿色平台 ---')
        self.execute_mission()


    def send_arm_pose(self, target_xyz, duration_sec):
        """控制手臂移动"""
        warm_start = [0.0] * 12
        warm_start[3], warm_start[5], warm_start[7] = -0.785, -2.356, 1.571
        
        joint_angles = self.chain.inverse_kinematics(target_xyz, 
                                                     target_orientation=[0, 0, -1], 
                                                     orientation_mode='Z', 
                                                     initial_position=warm_start)
        msg = JointTrajectory()
        msg.joint_names = [f'panda_joint{i}' for i in range(1, 8)]
        point = JointTrajectoryPoint()
        point.positions = list(joint_angles[2:9])
        point.time_from_start = Duration(sec=duration_sec, nanosec=0)
        msg.points = [point]
        self.arm_pub.publish(msg)

    def set_gripper(self, distance, duration_sec=1):
        """控制手指开合：0.04 是全开，0.0 是闭合"""
        msg = JointTrajectory()
        msg.joint_names = ['panda_finger_joint1', 'panda_finger_joint2']
        point = JointTrajectoryPoint()
        # 两个手指是对称移动的
        point.positions = [distance, distance]
        point.time_from_start = Duration(sec=duration_sec, nanosec=0)
        msg.points = [point]
        self.hand_pub.publish(msg)

    def execute_mission(self):
        # 准备工作：张开爪子
        self.get_logger().info('准备：张开爪子...')
        self.set_gripper(0.04) 
        time.sleep(2)

        # 1. 移动到方块上方
        self.get_logger().info('第一步：移动到方块上方...')
        self.send_arm_pose(self.pos_cube_above, 3)
        time.sleep(4)

        # 2. 下降
        self.get_logger().info('第二步：下降进行抓取...')
        self.send_arm_pose(self.pos_cube_grasp, 2)
        time.sleep(3)

        # 3. 抓取（闭合手指）
        self.get_logger().info('第三步：闭合爪子，实施抓取！')
        self.set_gripper(0.015) # 不要完全闭合到0，0.015刚好能捏紧10cm的方块
        time.sleep(2)

        # 补丁：确保“死死掐住”再起吊
        self.get_logger().info('>> 正在闭合爪子...')
        self.set_gripper(0.01)  # 发送夹紧指令

        # 关键：给物理引擎 3 秒时间来处理“挤压”和“静摩擦力”的建立
        # 此时机器人应该在原地不动，直到方块和手指完全接触并稳定
        time.sleep(3.0) 

        self.get_logger().info('>> 确认抓稳，准备起吊...')
        self.send_arm_pose(self.pos_cube_above, 2)

        # 4. 提起
        self.get_logger().info('第四步：提起方块...')
        self.send_arm_pose(self.pos_cube_above, 2)
        time.sleep(3)

        # 5. 转移到平台
        self.get_logger().info('第五步：向绿色平台转移...')
        self.send_arm_pose(self.pos_platform_above, 3)
        time.sleep(4)

        # 6. 放下
        self.get_logger().info('第六步：放下并张开爪子...')
        self.set_gripper(0.04)
        time.sleep(2)

        self.get_logger().info('任务圆满完成！')

def main():
    rclpy.init()
    node = PandaPickCommander()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()