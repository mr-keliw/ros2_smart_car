import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from order_interfaces.action import Order

import RPi.GPIO as GPIO
from .Motor import *


class LineTrackingActionServer(Node):

    def __init__(self):
        super().__init__("line_tracking_server")
        self._action_server = ActionServer(
            self, Order, "line_tracking_order", self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")

        feedback_msg = Order.Feedback()
        feedback_msg.en_route_observation = [0.0]

        self.IR01 = 14
        self.IR02 = 15
        self.IR03 = 23
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.IR01, GPIO.IN)
        GPIO.setup(self.IR02, GPIO.IN)
        GPIO.setup(self.IR03, GPIO.IN)

        self.PWM = Motor()

        # To get a value of 7 (0b111), all three inputs must be True:
        # self.LMR = 0 | 4 → self.LMR = 4
        # self.LMR = 4 | 2 → self.LMR = 6
        # self.LMR = 6 | 1 → self.LMR = 7
        t_end = time.monotonic() + goal_handle.request.order
        while time.monotonic() < t_end:
            self.LMR = 0x00
            # Left sensor
            if GPIO.input(self.IR01) == True:
                self.LMR = self.LMR | 4
            # Middle sensor
            if GPIO.input(self.IR02) == True:
                self.LMR = self.LMR | 2
            # Right sensor
            if GPIO.input(self.IR03) == True:
                self.LMR = self.LMR | 1
            # Forward
            if self.LMR == 2:
                self.PWM.setMotorModel(800, 800, 800, 800)
            # Turn left
            elif self.LMR == 4:
                self.PWM.setMotorModel(-2000, -2000, 4000, 4000)
            elif self.LMR == 6:
                self.PWM.setMotorModel(-1500, -1500, 2500, 2500)
            # Turn right
            elif self.LMR == 1:
                self.PWM.setMotorModel(4000, 4000, -2000, -2000)
            elif self.LMR == 3:
                self.PWM.setMotorModel(2500, 2500, -1500, -1500)
            # Pass
            elif self.LMR == 7:
                self.PWM.setMotorModel(0, 0, 0, 0)

            feedback_msg.en_route_observation[0] = self.LMR
            self.get_logger().info(
                "Feedback: {0}".format(feedback_msg.en_route_observation)
            )
            
        self.PWM.setMotorModel(0, 0, 0, 0)

        goal_handle.succeed()

        result = Order.Result()
        result.observation = feedback_msg.en_route_observation
        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = LineTrackingActionServer()

    rclpy.spin(action_server)


if __name__ == "__main__":
    main()
