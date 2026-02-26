# Client.py

import sys
from my_custom_interfaces.srv import AddThreeInts
import rclpy
from rclpy.node import Node


class AddThreeIntsClient(Node):
    def __init__(self):
        super().__init__('AddThreeIntsClient')
        self.client = self.create_client(
            AddThreeInts,
            'Add_Three_Ints'
        )
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, a, b, c):
        req = AddThreeInts.Request()
        req.a = a
        req.b = b
        req.c = c
        self.future = self.client.call_async(req)


def main():
    rclpy.init()
    if len(sys.argv) == 4:
        try:
            a = int(sys.argv[1])
            b = int(sys.argv[2])
            c = int(sys.argv[3])
        except ValueError:
            print('Usage: AddThreeIntsClient a b c')
            sys.exit(1)

        # send a request
        client = AddThreeIntsClient()
        client.send_request(a, b, c)

        while (rclpy.ok()):
            rclpy.spin_once(client)
            if client.future.done():
                try:
                    response = client.future.result()
                except Exception as e:
                    client.get_logger().info(f'Service call failed: {e}')
            else:
                client.get_logger().info(
                    f'Result of add three ints: [{a},{b},{c}] = [{response.sum}]'
                )
    else:
        print('Usage: AddThreeIntsClient a b c')
        sys.exit(1)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
