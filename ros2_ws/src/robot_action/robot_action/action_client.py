# action_client.py

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# custom class
from my_custom_interfaces.action import MoveTo


class RobotActionClient(Node):
    def __init__(self):
        super().__init__('robot_action_client')
        self._action_client = ActionClient(
            self,
            'move_to'
        )

    def send_goal(self, distance_to_move):
        goal_msg = MoveTo.Goal()
        goal_msg.target_distance = distance_to_move
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected...')
            return
        self.get_logger().info('Goal accepted...')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.goal_response_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal executed, total distance travelled: {result.total_distance_travelled}m')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback received: {feedback.current_distance_travelled}m')

def main(args=None):
    rclpy.init(args=args)
    robot_action_client = RobotActionClient()
    robot_action_client.send_goal(10.0)
    rclpy.spin(robot_action_client)
    robot_action_client.destroy_node()

if __name__ == '__main__':
    main()
