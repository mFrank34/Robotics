# add_two_ints_client.py

import sys
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(
            AddTwoInts,
            'add_two_ints'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'Service not available, waiting again...'
            )

    def send_request(self, a, b):
        # Create a new request with the integers to be added
        req = AddTwoInts.Request()
        req.a = a
        req.b = b
        # Async call to the service
        self.future = self.client.call_async(req)


def main():
    # Initialise the ROS client library
    rclpy.init()
    if len(sys.argv) == 3:
        try:
            # get the passed-in command-line arguments
            a = int(sys.argv[1])
            b = int(sys.argv[2])
        except ValueError:
            print('Usage: add_two_ints_client.py <int> <int>')
            sys.exit(1)
        # Create an instance of the service client
        client = AddTwoIntsClient()
        # Send a request with the provided integers
        client.send_request(a, b)

        while rclpy.ok():
            rclpy.spin_once(client)
            if client.future.done():
                try:
                    response = client.future.result()
                except Exception as e:
                    client.get_logger().info('Service call failed %r' % (e,))
            else:
                client.get_logger().info(f'Result of add_two_ints: {a} + {b} = {response.sum}')
    else:
        print('Usage: add_two_ints_client.py <int> <int>')
        sys.exit(1)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
