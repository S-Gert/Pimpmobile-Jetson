import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Int32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy 
from geometry_msgs.msg import PoseWithCovariance

import struct
import numpy as np
import pandas as pd
import threading
import serial
import time
from master.odometry import Odometry
import atexit
import pyudev
import time

from master.stanley_controller import stanley_controller, calculate_path_yaw

#arduino_nano_port = '/dev/ttyUSB0' # '/dev/ttyUSB1'
arduino_port = '/dev/ttyACM0'#'/dev/ttyACM0'
arduino = serial.Serial(port=arduino_port, baudrate=115200, timeout=1)
print("Opening port...")
time.sleep(2)
#arduino = try_ports(arduino_ports)

class Master(Node):

    def __init__(self, odometry_object):
        super().__init__('Master_Code')

        self.sub_lidar = self.create_subscription(
            Int32MultiArray,
            'lidar_pimp',
            self.lidar_callback,
            10)
        
        self.sub_gps = self.create_subscription(
            PoseWithCovariance,
            'gps_enu_pimp',
            self.gps_callback,
            10)
        
        self.sub_lidar 
        self.sub_gps

        # Arduino mega serial
        self.arduino_line = 0
        self.arduino_running = False

        # LIDAR
        self.obstacle = False

        # GPS variables
        self.stanley_running = True
        self.file_path: str = '/home/pimpmobile/ros2_ws/src/gps_publisher/gps_publisher/gps_log_data.csv'
        self.gps_df: pd.DataFrame = pd.read_csv(self.file_path)
        self.gps_x_col = self.gps_df["X"].values 
        self.gps_y_col = self.gps_df["Y"].values
        self.path_yaw = calculate_path_yaw(self.gps_x_col, self.gps_y_col) 

        self.gps_robot_x = 0
        self.gps_robot_y = 0
        self.gps_robot_z = 0
        
        self.gps_robot_last_x = 0
        self.gps_robot_last_y = 0

        self.k = 0.5
        self.v = 0.9 # 110 11.1ms
        self.k_s = 0.1
        self.target_index = 0
        self.previous_time = 0

        self.distance_tolerance = 0.15
        self.max_steering_control = np.radians(45)

    def lidar_callback(self, msg):
        lid_arr = msg.data
        if lid_arr[1] == 1 or lid_arr[2] == 1 or lid_arr[3] == 1:
            self.obstacle = True
        else:
            self.obstacle = False
    
    def gps_callback(self, msg) -> None:    
        self.gps_robot_x = msg.pose.position.x
        self.gps_robot_y = msg.pose.position.y
        self.gps_robot_z = msg.pose.position.z
        #print(f"x/y: {[self.gps_robot_x, self.gps_robot_y]}")

    def write_arduino(self, values: list) -> None:
        if len(values) != 3:
            raise ValueError("Arduino write list must be of 3 values")
        data = struct.pack('<3h', *values)
        arduino.write(data)
        
    def read_arduino(self):
        try:
            data = arduino.read(2)
            return struct.unpack('<h', data)[0]
        except:
            pass

    def stanley(self):
        """
        Runs on seperate thread, doing all the gps logic
        which includes reading the log file and sending commands to the robot.
        Normal = 1000
        GPS driving = 1500  
        GPS save = 2000
        """
        print(f"{self.path_yaw}")

        while self.stanley_running:
            yaw = np.arctan2(self.gps_robot_y - self.gps_robot_last_y, self.gps_robot_x - self.gps_robot_last_x)
            self.write_arduino([0, 0, 1])
            self.arduino_line = self.read_arduino()

            while self.target_index < len(self.gps_x_col) - 1 and not self.obstacle and self.arduino_line == 1500:
                
                yaw = np.arctan2(self.gps_robot_y - self.gps_robot_last_y, self.gps_robot_x - self.gps_robot_last_x)
                distance_threshold = 0.05
                if time.time() > self.previous_time + 0.2:
                    self.previous_time = time.time()
                    self.gps_robot_last_x, self.gps_robot_last_y = self.gps_robot_x, self.gps_robot_y

                limited_steering_angle, _, _ = stanley_controller(self.gps_robot_x, self.gps_robot_y, self.gps_x_col, self.gps_y_col, yaw, self.path_yaw, self.v, self.max_steering_control, self.k, self.k_s)
                #limited_steering_angle, target_index, crosstrack_error = stanley_controller(10, 10, self.gps_x_col, self.gps_y_col, yaw, self.path_yaw, self.v, self.max_steering_control, self.k, self.k_s)
                #print(f"no limit: {limited_steering_angle}")
                limited_steering_angle = int(limited_steering_angle * (255 / self.max_steering_control))

                #motor_speed = int() ...

                # self.read_arduino()
                self.write_arduino([90, -limited_steering_angle, 0])
                self.arduino_line = self.read_arduino()
                #print(f"inside: {limited_steering_angle}, x/y: {[self.gps_robot_x, self.gps_robot_y]}, closest point: {[self.gps_x_col[target_index]], self.gps_y_col[target_index]}, steering: {limited_steering_angle}")

        

@atexit.register
def exit_handler() -> None:
    arduino.close()
    print('Closing arduino port...')

def main(args=None):
    rclpy.init(args=args)
    odom_object = Odometry(64, 0.44, 0.155) # odometry object 64 ticks, 44 wheel distance, 15.5 wheel radius (m)
    m = Master(odom_object)


    ##### Multithreading arduino nano ###
    #encoder_read_thread = threading.Thread(target = m.read_arduino_nano_data)
    #encoder_read_thread.start()

    ##### Multithreading gps log file reading and logic #####
    stanley_thread = threading.Thread(target = m.stanley)
    stanley_thread.start()
    #####

    ##### Multithreading arduino reading ###
    # m.arduino_running = True
    # arduino_thread = threading.Thread(target = m.read_arduino)
    # arduino_thread.start()
    ####

    rclpy.spin(m)

    ##### Multithreading close ###
    #m.nano_running = False
    m.stanley_running = False
    # m.arduino_running = False
    #encoder_read_thread.join()
    stanley_thread.join()
    #arduino_thread.join()
    #####

    m.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
