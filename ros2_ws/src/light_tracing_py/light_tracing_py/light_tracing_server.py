import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from order_interfaces.action import Order

from .Motor import Motor
from .ADC import Adc


class LightTracingActionServer(Node):

    def __init__(self):
        super().__init__('light_tracing_server')
        self._action_server = ActionServer(
            self,
            Order,
            'light_tracing_order',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Order.Feedback()
        feedback_msg.en_route_observation = [0.0,0.0]

        adc=Adc()
        pwm=Motor()
        pwm.setMotorModel(0,0,0,0)

        t_end = time.monotonic() + goal_handle.request.order
        while time.monotonic() < t_end:
            left = adc.recvADC(0)
            right = adc.recvADC(1)

            feedback_msg.en_route_observation[0] = left
            feedback_msg.en_route_observation[1] = right
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.en_route_observation))
            goal_handle.publish_feedback(feedback_msg)

            if left < 2.99 and right < 2.99 :
                pwm.setMotorModel(600,600,600,600)
            elif abs(left-right)<0.15:
                pwm.setMotorModel(0,0,0,0)
            elif left > 3 or right > 3:
                if left > right :
                    pwm.setMotorModel(-1200,-1200,1400,1400)
                elif right > left :
                    pwm.setMotorModel(1400,1400,-1200,-1200)
        pwm.setMotorModel(0,0,0,0) 

        goal_handle.succeed()

        result = Order.Result()
        result.observation = feedback_msg.en_route_observation
        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = LightTracingActionServer()

    rclpy.spin(action_server)


if __name__ == '__main__':
    main()
