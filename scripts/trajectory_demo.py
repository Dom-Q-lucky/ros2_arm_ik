import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class PandaTrajectoryPlanner(Node):
    def __init__(self):
        super().__init__("oanda_trajectory_planner")

        self.publisher_ = self.create_publisher(JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.send_trajectory)
        self.get_logger().info("path planner is ready")

    def send_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = [f'panda_joint{i}' for i in range(1,8)]

        p1 = JointTrajectoryPoint()
        p1.positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785 ]
        p1.time_from_start = Duration(sec = 2, nanosec = 0)

        p2 = JointTrajectoryPoint()
        p2.positions = [0.5, -0.5, 0.2, -2.0, 0.5, 1.8, 1.0 ]
        p2.time_from_start = Duration(sec = 4, nanosec = 0)

        p3 = JointTrajectoryPoint()
        p3.positions = [-0.5, 0.0, -0.2, -1.5, -0.5, 2.0, 0.5]
        p3.time_from_start = Duration(sec = 6, nanosec = 0)

        msg.points = [p1, p2, p3]
        self.publisher_.publish(msg)
        self.get_logger().info('start to go')

        self.timer.cancel()

def main():
    rclpy.init()
    node = PandaTrajectoryPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()