# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from .Ultrasonic import Ultrasonic

class DistancePublisher(Node):

    def __init__(self):
        super().__init__('distance_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 1  # second(s)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def ultrasound(self):
        ultrasonic=Ultrasonic()
        return ultrasonic.get_distance()

    def timer_callback(self):
        msg = String()
        distance = self.ultrasound()
        msg.data = f'Distance: {distance}'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    distance_publisher = DistancePublisher()

    rclpy.spin(distance_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    distance_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
