# action_server.py

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

# custom interface that using
from my_custom_interfaces.action import MoveTo


class RobotActionServer(Node):
    """Class for sending robot actions."""
    def __init__(self):
        """Initialize the class and connect to action server."""
        super().__init__('robot_action_server')
        self._action_server = ActionServer(
            self, MoveTo,
            'move_to',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        """Execute the callback."""
        self.get_logger().info('Executing goal...')
        feedback_msg = MoveTo.Feedback()
        feedback_msg.current_distance_travelled = 0.0

        while feedback_msg.current_distance_travelled < goal_handle.request.target_distance:
            feedback_msg.current_distance_travelled += 1
            self.get_logger().info(f'Feedback: {feedback_msg.current_distance_travelled}')
            goal_handle.publish_feedback(feedback_msg)
            rclpy.spin_once(self, timeout_sec=1)

        goal_handle.succeed()

        # handle success
        result = MoveTo.Result()
        result.total_distance_travelled = feedback_msg.current_distance_travelled
        return result

def main(args=None):
    """Start robot action server."""
    rclpy.init(args=args)
    robot_action_server = RobotActionServer()
    print("Action Server is Running...")
    try:
        while rclpy.ok():
            rclpy.spin_once(robot_action_server)
    except KeyboardInterrupt:
        robot_action_server._action_server.destroy()
        robot_action_server.destroy_node()

if __name__ == '__main__':
    main()