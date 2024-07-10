from order_interfaces.srv import Order

import rclpy
from rclpy.node import Node

import time
from .Motor import *
from .ADC import *
from .Line_Tracking import *
from .Led import *
from .Buzzer import *
from .servo import *
from .Ultrasonic import *


class ModuleTestService(Node):

    def __init__(self):
        super().__init__("minimal_service")
        self.srv = self.create_service(Order, "test_order", self.order_callback)

    def order_callback(self, request, response):
        if request.order == 1:
            pwm = Motor()

            try:
                # Forward
                pwm.setMotorModel(1000, 1000, 1000, 1000)
                self.get_logger().info("The car is driving forward.")
                time.sleep(1)
                # Backward
                pwm.setMotorModel(-1000, -1000, -1000, -1000)
                self.get_logger().info("The car is reversing.")
                time.sleep(1)
                # Turn left
                pwm.setMotorModel(-1500, -1500, 2000, 2000)
                self.get_logger().info("The car is making a left turn.")
                time.sleep(1)
                # Turn right
                pwm.setMotorModel(2000, 2000, -1500, -1500)
                self.get_logger().info("The car is making a right turn.")
                time.sleep(1)
                # Move left
                pwm.setMotorModel(-2000, 2000, 2000, -2000)
                self.get_logger().info("The car is sliding sideways to the left.")
                time.sleep(1)
                # Move right
                pwm.setMotorModel(2000, -2000, -2000, 2000)
                self.get_logger().info("The car is sliding sideways to the right.")
                time.sleep(1)

                # Move diagonally to the left and forward
                pwm.setMotorModel(0, 2000, 2000, 0)
                self.get_logger().info(
                    "The car is moving forward and to the left diagonally."
                )
                time.sleep(1)
                # Move diagonally to the right and backward
                pwm.setMotorModel(0, -2000, -2000, 0)
                self.get_logger().info(
                    "The car is moving backward and to the right diagonally."
                )
                time.sleep(1)
                # Move diagonally to the right and forward
                pwm.setMotorModel(2000, 0, 0, 2000)
                self.get_logger().info(
                    "The car is moving forward and to the right diagonally."
                )
                time.sleep(1)
                # Move diagonally to the left and backward
                pwm.setMotorModel(-2000, 0, 0, -2000)
                self.get_logger().info(
                    "The car is moving backward and to the left diagonally."
                )
                time.sleep(1)

                # Stop
                pwm.setMotorModel(0, 0, 0, 0)
                self.get_logger().info("\nThe test is complete.")
                response.result = 1
            except KeyboardInterrupt:
                pwm.setMotorModel(0, 0, 0, 0)
                self.get_logger().info("\nThe test is complete.")
                response.result = 0

        elif request.order == 2:
            pwm = Motor()

            try:
                pwm.Rotate(0)
                response.result = 1
            except KeyboardInterrupt:
                self.get_logger().info("\nThe test is complete.")
                response.result = 0

        elif request.order == 3:
            adc = Adc()

            try:
                t_end = 5
                while t_end > 0:
                    left_adc = adc.recvADC(0)
                    self.get_logger().info(
                        f"The voltage from the photoresistor on the left is {left_adc} V"
                    )
                    right_adc = adc.recvADC(1)
                    self.get_logger().info(
                        f"The voltage from the photoresistor on the right is {right_adc} V"
                    )
                    Power = adc.recvADC(2)
                    self.get_logger().info(f"The battery voltage is {Power * 3} V")
                    time.sleep(1)
                    self.get_logger().info("\n")
                    t_end -= 1
                response.result = 1
            except KeyboardInterrupt:
                self.get_logger().info("\nThe test is complete.")
                response.result = 0

        elif request.order == 4:
            line = Line_Tracking()

            try:
                t_end = time.monotonic() + 10
                while time.monotonic() < t_end:
                    if (
                        GPIO.input(line.IR01) != True
                        and GPIO.input(line.IR02) == True
                        and GPIO.input(line.IR03) != True
                    ):
                        self.get_logger().info("Middle")
                    elif (
                        GPIO.input(line.IR01) != True
                        and GPIO.input(line.IR02) != True
                        and GPIO.input(line.IR03) == True
                    ):
                        self.get_logger().info("Right")
                    elif (
                        GPIO.input(line.IR01) == True
                        and GPIO.input(line.IR02) != True
                        and GPIO.input(line.IR03) != True
                    ):
                        self.get_logger().info("Left")
                response.result = 1
            except KeyboardInterrupt:
                self.get_logger().info("\nThe test is complete.")
                response.result = 0

        elif request.order == 5:
            led = Led()

            try:
                led.ledIndex(0x01, 255, 0, 0) # red
                led.ledIndex(0x02, 255, 125, 0) # orange
                led.ledIndex(0x04, 255, 255, 0) # yellow
                led.ledIndex(0x08, 0, 255, 0) # green
                led.ledIndex(0x10, 0, 255, 255) # cyan-blue
                led.ledIndex(0x20, 0, 0, 255) # blue
                led.ledIndex(0x40, 128, 0, 128) # purple
                led.ledIndex(0x80, 255, 255, 255) # white
                self.get_logger().info(
                    "The LEDs are lit, displaying the colors red, orange, yellow, green, cyan-blue, blue, and white."
                )
                time.sleep(3)  # wait 3s
                led.colorWipe(led.strip, Color(0, 0, 0)) # turn off the lights
                self.get_logger().info("\nThe test is complete.")
                response.result = 1
            except KeyboardInterrupt:
                led.colorWipe(led.strip, Color(0, 0, 0)) # turn off the lights
                self.get_logger().info("\nThe test is complete.")
                response.result = 0

        elif request.order == 6:
            buzzer = Buzzer()

            try:
                buzzer.run("1")
                time.sleep(1)
                self.get_logger().info("1 second")
                time.sleep(1)
                self.get_logger().info("2 seconds")
                time.sleep(1)
                self.get_logger().info("3 seconds")
                buzzer.run("0")
                self.get_logger().info("\nThe test is complete.")
                response.result = 1
            except KeyboardInterrupt:
                buzzer.run("0")
                self.get_logger().info("\nThe test is complete.")
                response.result = 0

        elif request.order == 7:
            pwm = Servo()

            try:
                for i in range(50, 110, 1):
                    pwm.setServoPwm("0", i)
                    time.sleep(0.01)
                for i in range(110, 50, -1):
                    pwm.setServoPwm("0", i)
                    time.sleep(0.01)
                for i in range(80, 150, 1):
                    pwm.setServoPwm("1", i)
                    time.sleep(0.01)
                for i in range(150, 80, -1):
                    pwm.setServoPwm("1", i)
                    time.sleep(0.01)

                pwm.setServoPwm("0", 90)
                pwm.setServoPwm("1", 90)

                response.result = 1
            except KeyboardInterrupt:
                pwm.setServoPwm("0", 90)
                pwm.setServoPwm("1", 90)
                self.get_logger().info("\nThe test is complete.")
                response.result = 0

        elif request.order == 8:
            ultrasonic = Ultrasonic()
            try:
                t_end = 5
                while t_end > 0:
                    data = ultrasonic.get_distance()
                    self.get_logger().info(f"The obstacle is {data} cm away.")
                    time.sleep(1)
                    t_end -= 1
                response.result = 1
            except KeyboardInterrupt:
                self.get_logger().info("\nThe test is complete.")
                response.result = 0

        else:
            response.result = 0

        self.get_logger().info("Incoming request\norder: %d " % (request.order))

        return response


def main():
    rclpy.init()

    module_test_service = ModuleTestService()

    rclpy.spin(module_test_service)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
