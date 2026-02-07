# SpeedPublisher.py

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SpeedPublisher(Node):
    """Subscribe to `rpm` (Float32) and publish linear `speed` (Float32).
    Converts rpm -> m/s using linear_speed = rpm * 2*pi*radius / 60.
    Wheel radius can be configured via the `wheel_radius` parameter (meters).
    """

    def __init__(self):
        super().__init__('speed_publisher')

        self.declare_parameter('wheel_radius', 0.033)
        self.wheel_radius = self.get_parameter('wheel_radius').value

        self.subscription = self.create_subscription(
            Float32,
            'rpm',
            self.listener_callback,
            10)

        self.publisher_ = self.create_publisher(Float32, 'speed', 10)

        self.get_logger().info(f'SpeedPublisher started (wheel_radius={self.wheel_radius} m)')

    def listener_callback(self, msg: Float32):
        rpm = float(msg.data)
        linear_speed = rpm * 2.0 * math.pi * float(self.wheel_radius) / 60.0

        out = Float32()
        out.data = float(linear_speed)

        self.publisher_.publish(out)
        self.get_logger().info(f'RPM: {rpm:.2f} -> speed: {linear_speed:.3f} m/s')


def main(args=None):
    rclpy.init(args=args)
    node = SpeedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()