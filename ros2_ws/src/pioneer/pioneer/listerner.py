import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class Listener(Node):
    """Subscribe to both `rpm` and `speed` topics and log received values."""

    def __init__(self):
        super().__init__('dual_listener')

        self.last_rpm = None
        self.last_speed = None

        self.create_subscription(Float32, 'rpm', self.rpm_callback, 10)
        self.create_subscription(Float32, 'speed', self.speed_callback, 10)

        self.get_logger().info('DualListener started: subscribing to /rpm and /speed')

    def rpm_callback(self, msg: Float32):
        self.last_rpm = float(msg.data)
        self.get_logger().info(f'Received rpm: {self.last_rpm:.2f}')
        self._maybe_report()

    def speed_callback(self, msg: Float32):
        self.last_speed = float(msg.data)
        self.get_logger().info(f'Received speed: {self.last_speed:.3f} m/s')
        self._maybe_report()

    def _maybe_report(self):
        if self.last_rpm is None or self.last_speed is None:
            return
        self.get_logger().info(f'Combined reading -> rpm: {self.last_rpm:.2f}, speed: {self.last_speed:.3f} m/s')


def main(args=None):
    rclpy.init(args=args)
    node = DualListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
