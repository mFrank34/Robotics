# Server.py

import rclpy
from rclpy.node import Node
from my_custom_interfaces.srv import AddThreeInts


class AddThreeIntsServer(Node):
    def __init__(self):
        super().__init__('AddThreeIntsServer')
        self.service = self.create_service(
            AddThreeInts,
            'Add_Three_Ints',
            self.add_three_ints_callback
        )

    def add_three_ints_callback(self, request, response):
        response.sum = request.a + request.b + request.c
        self.get_logger().info(
            f'Receiving request: a = {request.a}, b = {request.b}, c = {request.c} (sum: {response.sum})')
        return response

def main(args=None):
    rclpy.init(args=args)
    server = AddThreeIntsServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
