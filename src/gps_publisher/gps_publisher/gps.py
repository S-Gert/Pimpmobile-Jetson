import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import pynmea2
import time

gps_port = '/dev/ttyACM0'
baudrate = 9600

try:
    ser = serial.Serial(gps_port, baudrate, timeout=1)
except serial.SerialException as e:
    print("Error:", e)

class GpsPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(String, 'gps_data', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def get_gps_coordinates(self):
        line = ser.readline().decode('utf-8')
        if line.startswith('$GNGGA'):
            msg = pynmea2.parse(line)
            latitude = msg.latitude
            longitude = msg.longitude
            return f"lat: {latitude}, long: {longitude}"
        return "0"

    def timer_callback(self):
        msg = String()
        msg.data = self.get_gps_coordinates()
        if msg.data != "0":
            self.publisher_.publish(msg)
            self.get_logger().info('"%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GpsPublisher()
    rclpy.spin(gps_publisher)
    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()