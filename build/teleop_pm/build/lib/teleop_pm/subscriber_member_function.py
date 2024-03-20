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
import serial
import time
from std_msgs.msg import String

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=.1)


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'teleop_pimpmobile',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def write_data(self, x):
        arduino.flushInput()
        arduino.write(bytes(x.data, 'utf-8'))
        time.sleep(0.05)
        data = arduino.readline()
        return data

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.write_data(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()