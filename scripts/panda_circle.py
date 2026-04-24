import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import ikpy.chain
import os
import numpy as np
import math

class PandaCircleCommander(Node):
    def __init__(self):
        super().__init__('panda_circle_commander', 
                         parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        self.publisher_ = self.create_publisher(JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        
        # 1. 初始化 IK 链（确保路径正确）
        urdf_path = os.path.expanduser('~/panda_ws/src/panda_description/urdf/panda_fixed.urdf')
        self.chain = ikpy.chain.Chain.from_urdf_file(urdf_path, base_elements=["world"])
        for i, link in enumerate(self.chain.links):
            link.is_kinematic = (2 <= i <= 8) # 锁定非动力关节

        # 2. 设定参数
        self.center = [0.4, 0.0, 0.7] # 圆心：前方 40cm，高度 70cm
        self.radius = 0.1             # 半径：10cm
        self.angle = 0.0              # 初始角度
        
        # 3. 初始猜测位姿（热启动）
        self.warm_start = [0.0] * 12
        self.warm_start[3], self.warm_start[5], self.warm_start[7] = -0.785, -2.356, 1.571

        # 4. 定时器：每 0.1 秒计算一次新位置（10Hz 刷新，保证丝滑）
        self.timer = self.create_timer(0.1, self.draw_step)
        self.get_logger().info('Panda 绘画大师已上线，开始画圆...')

    def draw_step(self):
        # 计算当前圆周坐标
        x = self.center[0] + self.radius * math.cos(self.angle)
        y = self.center[1] + self.radius * math.sin(self.angle)
        z = self.center[2]
        
        # 逆解计算
        joint_angles = self.chain.inverse_kinematics([x, y, z], initial_position=self.warm_start)
        final_angles = list(joint_angles[2:9])

        # 发送指令
        msg = JointTrajectory()
        msg.joint_names = [f'panda_joint{i}' for i in range(1, 8)]
        
        point = JointTrajectoryPoint()
        point.positions = final_angles
        point.time_from_start = Duration(sec=0, nanosec=100000000) # 0.1秒执行时间
        
        msg.points = [point]
        self.publisher_.publish(msg)
        
        # 更新角度，准备下一步（每一圈约 6.28 秒跑完）
        self.angle += 0.1 
        if self.angle > 2 * math.pi:
            self.angle = 0.0
            self.get_logger().info('完成一圈绘画！')

def main():
    rclpy.init()
    node = PandaCircleCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()