import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import String
from lidar.lidar import LidarSubscriber

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=.1)

class TeleopSerial(Node):

    def __init__(self):
        super().__init__('teleop_serial')
        self.subscription = self.create_subscription(
            String,
            'teleop_pimpmobile',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.lidar_obj = LidarSubscriber()

    def write_data(self, x):
        arduino.flushInput()
        arduino.write(bytes(x.data, 'utf-8'))
        time.sleep(0.05)
        data = arduino.readline()
        return data

    def read_lidar(self):
        return lidar_obj.obstacle_array

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        if read_lidar():
            self.write_data("0, 0, 500")
        self.write_data(msg)


def main(args=None):
    rclpy.init(args=args)

    teleop_serial = TeleopSerial()

    rclpy.spin(teleop_serial)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    teleop_serial.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
