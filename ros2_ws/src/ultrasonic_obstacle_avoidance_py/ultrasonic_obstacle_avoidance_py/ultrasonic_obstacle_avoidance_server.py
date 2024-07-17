import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from order_interfaces.action import Order

import RPi.GPIO as GPIO
from .Motor import Motor
from .servo import Servo


class UltrasonicObstacleAvoidanceActionServer(Node):

    def __init__(self):
        super().__init__('ultrasonic_obstacle_avoidance_server')
        self._action_server = ActionServer(
            self,
            Order,
            'ultrasonic_obstacle_avoidance_order',
            self.execute_callback)

    def pulse_in(self, pin, level, timeout):  # Measure the pulse duration of a pin with a timeout.
        t0 = time.time()
        while (GPIO.input(pin) != level):
            if ((time.time() - t0) > timeout * 0.000001):
                return 0
        t0 = time.time()
        while (GPIO.input(pin) == level):
            if ((time.time() - t0) > timeout * 0.000001):
                return 0
        pulse_time = (time.time() - t0) * 1000000
        return pulse_time

    def get_distance(self):  # Obtain the measurement results from the ultrasonic module in centimeters.
        distance_cm = [0, 0, 0, 0, 0]
        for i in range(5):
            GPIO.output(self.TRIGGER_PIN, GPIO.HIGH)  # Set the TRIGGER_PIN to output a HIGH signal for 10 microseconds.
            time.sleep(0.00001)  # 10 microseconds
            GPIO.output(self.TRIGGER_PIN, GPIO.LOW)  # Set the TRIGGER_PIN to output a LOW signal.
            ping_time = self.pulse_in(self.ECHO_PIN, GPIO.HIGH, self.TIMEOUT)  # Read the pulse duration of the ECHO_PIN.
            distance_cm[i] = ping_time * 340.0 / 2.0 / 10000.0  # Calculate the distance using a sound speed of 340 m/s.
        distance_cm.sort()
        return distance_cm[2]
    
    def run_motor(self, left_d, front_d, right_d):
        if (left_d < 30 and front_d < 30 and right_d < 30) or front_d < 30:
            # Backward
            self.pwm.setMotorModel(-1000, -1000, -1000, -1000)
            time.sleep(0.1)
            if left_d < right_d:
                # Turn right
                self.pwm.setMotorModel(1000, 1000, -1000, -1000)
            else:
                # Turn left
                self.pwm.setMotorModel(-1000, -1000, 1000, 1000)
        elif left_d < 30 and front_d < 30:
            # Turn right
            self.pwm.setMotorModel(1500, 1500, -1500, -1500)
        elif right_d < 30 and front_d < 30:
            # Turn left
            self.pwm.setMotorModel(-1500, -1500, 1500, 1500)
        elif left_d < 20:
            # Turn right
            self.pwm.setMotorModel(1500, 1500, -500, -500)
            if left_d < 10:
                self.pwm.setMotorModel(1500, 1500, -1000, -1000)
        elif right_d < 20:
            # Turn left
            self.pwm.setMotorModel(-500, -500, 1500, 1500)
            if right_d < 10:
                self.pwm.setMotorModel(-1000, -1000, 1000, 1000)
        else:
            # Forward
            self.pwm.setMotorModel(600, 600, 600, 600)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Order.Feedback()
        feedback_msg.en_route_observation = [0.0, 0.0, 0.0]

        GPIO.setwarnings(False)
        self.TRIGGER_PIN = 27
        self.ECHO_PIN = 22
        self.MAX_DISTANCE = 300  # Set the maximum measuring distance in centimeters.
        self.TIMEOUT = self.MAX_DISTANCE * 60  # Calculate the timeout based on the maximum measuring distance.
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.TRIGGER_PIN, GPIO.OUT)
        GPIO.setup(self.ECHO_PIN, GPIO.IN)

        self.pwm = Motor()
        self.pwm_s = Servo()

        t_end = time.monotonic() + goal_handle.request.order
        while time.monotonic() < t_end:
            self.pwm_s.setServoPwm("0", 90)
            time.sleep(0.1)
            front_d = self.get_distance()

            if front_d < 30:
                self.pwm_s.setServoPwm("0", 30)
                time.sleep(0.2)
                left_d = self.get_distance()
                self.pwm_s.setServoPwm("0", 151)
                time.sleep(0.2)
                right_d = self.get_distance()

                feedback_msg.en_route_observation[0] = left_d
                feedback_msg.en_route_observation[1] = front_d
                feedback_msg.en_route_observation[2] = right_d
                self.get_logger().info('Feedback: {0}'.format(feedback_msg.en_route_observation))

                self.run_motor(left_d, front_d, right_d)
                self.pwm_s.setServoPwm("0", 90)
            else:
                feedback_msg.en_route_observation[1] = front_d
                self.get_logger().info('Feedback: {0}'.format(feedback_msg.en_route_observation))

                self.run_motor(20, front_d, 20)

        self.pwm.setMotorModel(0,0,0,0)

        goal_handle.succeed()

        result = Order.Result()
        result.observation = feedback_msg.en_route_observation
        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = UltrasonicObstacleAvoidanceActionServer()

    rclpy.spin(action_server)


if __name__ == '__main__':
    main()
