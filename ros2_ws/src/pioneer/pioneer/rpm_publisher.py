# RpmPublisher.py
import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float

class RpmPublisher(Node):
    def __init__(self):
        super().__init__('rpm_publisher')
        self.publisher_ = self.create_publisher(Float, 'rpm', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.rpm = 0.0

    def timer_callback(self):
        msg = Float()
        msg.data = self.rpm
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing RPM: "%f"' % msg.data)
        self.rpm += 10.0 # Increment RPM for demonstration

def main(args=None):
    rclpy.init(args=args)
    rpm_publisher = RpmPublisher()
    rclpy.spin(rpm_publisher)
    rpm_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()