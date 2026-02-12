# add_two_ints_server.py
from example_interfaces.srv import AddTwoInts
from rclpy.node import Node


class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.service = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.callback
        )
        self.get_logger().info(
            'add_two_ints service is running'
        )

    def add_two_ints_callback(
            self,
            request,
            response
    ):
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Receiving request: a = {request.a}, b = {request.b} and sum = {response.sum}'
        )
        return response

def main(args=None):
    rclpy.init(args=args)
    server = AddTwoIntsServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()