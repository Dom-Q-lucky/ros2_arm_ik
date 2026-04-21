import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import ikpy.chain
import os
import numpy as np

class PandaIKCommander(Node):
    def __init__(self):
        # 【关键点 1】必须开启 use_sim_time，否则 Gazebo 会无视你的指令
        super().__init__('panda_ik_commander', 
                         parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        self.publisher_ = self.create_publisher(JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        
        # 加载 IK 链
        urdf_path = os.path.expanduser('~/panda_ws/src/panda_description/urdf/panda_fixed.urdf')
        self.chain = ikpy.chain.Chain.from_urdf_file(urdf_path, base_elements=["world"])
        
        # 锁定非动力关节
        for i, link in enumerate(self.chain.links):
            link.is_kinematic = (2 <= i <= 8)

        # 【关键点 2】设置一个“热启动”姿态，防止求解器摆烂
        self.warm_start_pose = [0.0] * 12
        self.warm_start_pose[3] = -0.785   # joint 2
        self.warm_start_pose[5] = -2.356   # joint 4
        self.warm_start_pose[7] = 1.571    # joint 6

        self.timer = self.create_timer(2.0, self.perform_movement)
        self.target_z = 0.7
        self.get_logger().info('空间指挥官已就位，使用热启动模式控制...')

    def perform_movement(self):
        # 切换高度
        self.target_z = 0.85 if self.target_z == 0.7 else 0.7
        # 设定一个它够得着的坐标
        target_pos = [0.4, 0.1, self.target_z]
        
        # 【关键点 3】计算逆解时传入初始姿态
        joint_angles = self.chain.inverse_kinematics(
            target_position=target_pos,
            initial_position=self.warm_start_pose 
        )
        final_angles = list(joint_angles[2:9])

        # 发送轨迹
        msg = JointTrajectory()
        msg.joint_names = [f'panda_joint{i}' for i in range(1, 8)]
        
        point = JointTrajectoryPoint()
        point.positions = final_angles
        # 稍微给多点时间，让它动得平滑些
        point.time_from_start = Duration(sec=1, nanosec=500000000)
        
        msg.points = [point]
        self.publisher_.publish(msg)
        self.get_logger().info(f'已发送逆解指令，目标高度: {self.target_z}')

def main():
    rclpy.init()
    node = PandaIKCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()