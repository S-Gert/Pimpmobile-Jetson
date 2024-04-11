import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseWithCovariance
import pymap3d as pm

import socket
import serial
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
        self.publisher_raw_ = self.create_publisher(NavSatFix, 'gps_raw_data_pimp', 10)
        timer_period = 0.05  # seconds
        self.timer_raw = self.create_timer(timer_period, self.gps_callback)

        self.publisher_enu_ = self.create_publisher(PoseWithCovariance, 'gps_enu_pimp', 11)
        self.timer_enu = self.create_timer(timer_period, self.enu_callback)

        # Baasjaam
        self.lock_zero_point = False
        self.lat0 = 58.3428685594
        self.lon0 = 25.5692475361
        self.alt0 = 91.357      

    def gps_base_tcp(self):
        s = socket.socket()
        port = 8002
        s.connect(('213.168.5.170', port))
        return s.recv(1024)
    
    def status_fix(self):
        try:
            socket_info = self.gps_base_tcp()
            ser.write(socket_info)
        except Exception as e:
            self.get_logger().error(f"Error writing to u-blox device: {e}")

    def transform_to_enu(self, lat, lon, alt):
        # e,n,u - east, north, up rtcm
        x,y,z = pm.geodetic2enu(lat, lon, alt, self.lat0, self.lon0, self.alt0)
        return x, y, z
    
    def convert(self, coord):
        degrees = int(coord / 100)
        minutes = (coord % 100)
        result = degrees + minutes / 60
        return result
    
    def enu_callback(self):
        line = ser.readline().decode('utf-8')
        line_split = line.split(",")
        if float(line_split[6]) > 3:
            if not self.lock_zero_point:
                self.lat0 = self.convert(float(line_split[2]))
                self.lon0 = self.convert(float(line_split[4]))
                self.alt0 = float(line_split[9])
                self.lock_zero_point = True
            enu = PoseWithCovariance()
            x, y, z = self.transform_to_enu(self.convert(float(line_split[2])), self.convert(float(line_split[4])), float(line_split[9]))
            enu.pose.position.x = x
            enu.pose.position.y = y
            enu.pose.position.z = z
            self.publisher_enu_.publish(enu)
            #self.get_logger().info(f"{x}, {y}, {z}")
        
    def gps_callback(self):
        try:
            line = ser.readline().decode('utf-8')
            line_split = line.split(",")
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gps"
            msg.latitude = float(line_split[2])
            msg.longitude = float(line_split[4])
            msg.altitude = float(line_split[9])
            msg.status.status = int(line_split[6])
            if msg.status.status < 4:
                self.status_fix()
            self.publisher_raw_.publish(msg)
            #self.get_logger().info(f"{line_split}")
        except:
            self.get_logger().error(f"Error (no GPS signal)")


def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GpsPublisher()
    rclpy.spin(gps_publisher)
    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()