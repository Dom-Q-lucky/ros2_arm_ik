import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer

class PandaPoseListener(Node):
    def __init__(self):
        super().__init__('panda_pose_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.get_pose)

    def get_pose(self):
        try:
            # 查找从 world 到末端 link8 的坐标变换
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('world', 'panda_link8', now)
            
            p = trans.transform.translation
            self.get_logger().info(f'当前末端坐标: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}')
        except Exception as e:
            self.get_logger().warn('坐标获取失败，可能变换还没建立...')

def main():
    rclpy.init()
    node = PandaPoseListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()