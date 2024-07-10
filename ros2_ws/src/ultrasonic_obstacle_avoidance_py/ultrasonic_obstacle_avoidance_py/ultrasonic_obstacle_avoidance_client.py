import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from order_interfaces.action import Order


class UltrasonicObstacleAvoidanceActionClient(Node):

    def __init__(self):
        super().__init__('ultrasonic_obstacle_avoidance_client')
        self._action_client = ActionClient(self, Order, 'ultrasonic_obstacle_avoidance_order')

    def send_goal(self, order):
        goal_msg = Order.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.observation))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.en_route_observation))


def main(args=None):
    rclpy.init(args=args)

    action_client = UltrasonicObstacleAvoidanceActionClient()

    action_client.send_goal(int(sys.argv[1]))

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
