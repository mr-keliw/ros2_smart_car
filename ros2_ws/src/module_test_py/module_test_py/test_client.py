import sys

from order_interfaces.srv import Order
import rclpy
from rclpy.node import Node


class ModuleTestClientAsync(Node):

    def __init__(self):
        super().__init__("module_test_client_async")
        self.cli = self.create_client(Order, "test_order")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = Order.Request()

    def send_request(self, o):
        self.req.order = o
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    module_test_client = ModuleTestClientAsync()
    future = module_test_client.send_request(int(sys.argv[1]))
    rclpy.spin_until_future_complete(module_test_client, future)
    response = future.result()
    module_test_client.get_logger().info(
        "Result of order: order %d, result %d " % (int(sys.argv[1]), response.result)
    )

    module_test_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
