import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from order_interfaces.action import Order

from .Motor import *
from .ADC import *


class LightTracingActionServer(Node):

    def __init__(self):
        super().__init__('movement_action_server')
        self._action_server = ActionServer(
            self,
            Order,
            'order',
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
            L = adc.recvADC(0)
            R = adc.recvADC(1)

            feedback_msg.en_route_observation[0] = L
            feedback_msg.en_route_observation[1] = R
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.en_route_observation))
            goal_handle.publish_feedback(feedback_msg)

            if L < 2.99 and R < 2.99 :
                pwm.setMotorModel(600,600,600,600)
            elif abs(L-R)<0.15:
                pwm.setMotorModel(0,0,0,0)
            elif L > 3 or R > 3:
                if L > R :
                    pwm.setMotorModel(-1200,-1200,1400,1400)
                elif R > L :
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
