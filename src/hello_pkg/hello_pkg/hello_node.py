import rclpy
from rclpy.node import Node


class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.counter = 0
        self.timer = self.create_timer(1.0, self.on_timer)
        self.get_logger().info('hello_node started')

    def on_timer(self):
        self.counter += 1
        self.get_logger().info(f'Hello from hello_pkg #{self.counter}')


def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
