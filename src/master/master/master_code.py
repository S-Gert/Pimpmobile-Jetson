import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Int32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy 
from geometry_msgs.msg import PoseWithCovariance
from master.gstreamer_host import Gstream

import struct
import numpy as np
import pandas as pd
import threading
import serial
import time
from master.odometry import Odometry
import atexit
import math

from master.stanley_controller import stanley_controller, calculate_path_yaw

#arduino_nano_port = '/dev/ttyUSB0' # '/dev/ttyUSB1'
arduino_port = '/dev/ttyACM0'#'/dev/ttyACM0'
arduino = serial.Serial(port=arduino_port, baudrate=115200, timeout=1)
print("Opening port...")
time.sleep(2)

class Master(Node):

    def __init__(self, odometry_object):
        super().__init__('Master_Code')

        # Start gstreamer pipeline
        self.gstream_obj = Gstream()
        self.gstream_obj.start_stream()

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
        
        self.sub_to = self.create_subscription(
            Int32MultiArray,
            'pimpmobile_teleop',
            self.teleop_callback,
            10)
        
        self.sub_lidar 
        self.sub_gps

        # Arduino mega serial
        self.arduino_line = 0

        # LIDAR
        self.obstacle = False

        # Teleop variables
        self.to_motors = 0
        self.to_servo = 0
        self.to_brakes = 0
        self.to_reverse = 0
        self.to_speed_limiter = 100
        self.to_handbrake_toggle = 0
        self.to_gps_saving_toggle = 0
        self.to_stanley_reset = 0
        self.to_stanley_drive_toggle = 0

        # Camera UI variables
        self.last_update_time = 0

        # GPS variables
        self.file_path: str = '/home/pimpmobile/ros2_ws/src/gps_publisher/gps_publisher/gps_log_data.csv'
        self.gps_df: pd.DataFrame = pd.read_csv(self.file_path)
        self.gps_x_col = self.gps_df["X"].values 
        self.gps_y_col = self.gps_df["Y"].values
        self.path_yaw = calculate_path_yaw(self.gps_x_col, self.gps_y_col) 

        self.gps_robot_x = 0
        self.gps_robot_y = 0
        self.gps_robot_z = 0
        
        self.gps_robot_last_x, self.gps_robot_last_y = 0, 0
        self.gps_speed_last_x, self.gps_speed_last_y = 0, 0

        # Stanley variables
        self.at_final_point = False
        self.yaw = 0
        self.mainloop_running = True
        self.k = 0.7
        self.v = 1.5 # 110 11.1ms
        self.k_s = 0.1
        self.target_index = 0
        self.absolute_distance = 0
        self.previous_time = 0

        self.distance_tolerance = 0.15
        self.max_steering_control = np.radians(45)

    def lidar_callback(self, msg):
        lid_arr = msg.data
        if lid_arr[1] == 1 or lid_arr[2] == 1 or lid_arr[3] == 1:
            self.obstacle = True
        else:
            self.obstacle = False
        #print(f"{self.obstacle}, {lid_arr}")
    
    def gps_callback(self, msg) -> None:    
        self.gps_robot_x = msg.pose.position.x
        self.gps_robot_y = msg.pose.position.y
        self.gps_robot_z = msg.pose.position.z

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

    def teleop_callback(self, msg):
        self.to_motors = msg.data[0]
        self.to_servo = msg.data[1]
        self.to_brakes = msg.data[2]
        self.to_reverse = msg.data[3]
        if self.to_reverse == 1:
            self.to_motors = -self.to_motors
        self.to_speed_limiter = msg.data[4]
        self.to_handbrake_toggle = msg.data[5]
        if self.to_handbrake_toggle == 1:
            self.to_motors = 0
            self.to_brakes = 1
        self.to_gps_saving_toggle = msg.data[6]
        self.to_stanley_reset = msg.data[7]
        if self.to_stanley_reset == 1:
            self.at_final_point = False
            #TODO: previous target index = 0
        self.to_stanley_drive_toggle = msg.data[8]

    def check_if_at_final_point(self):
        if self.target_index > len(self.gps_x_col) - 4:
            if self.absolute_distance < 0.8:
                self.at_final_point = True
        self.at_final_point = False

    def update_camera_ui(self):
        ui_update_rate = 0.5 # seconds
        self.gstream_obj.speed_limiter_text = self.to_speed_limiter
        self.gstream_obj.gps_status_text = f"X:{self.gps_robot_x}, Y:{self.gps_robot_y}"

        if self.at_final_point == True:
            self.gstream_obj.stanley_at_final_point_text = "Path complete"
        else:
            self.gstream_obj.stanley_at_final_point_text = "Not complete"

        if self.to_stanley_drive_toggle == 1:
            self.gstream_obj.stanley_running_text = "Running"
        else:
            self.gstream_obj.stanley_running_text = "Not running"

        if self.to_handbrake_toggle == 1:
            self.gstream_obj.current_gear_text = "HB"
        elif self.to_reverse == 1:
            self.gstream_obj.current_gear_text = "R"
        else:
            self.gstream_obj.current_gear_text = "D"
        if time.time() > self.last_update_time + ui_update_rate:
            distance = 0
            if self.gps_robot_x - self.gps_speed_last_x != 0 and self.gps_robot_y - self.gps_speed_last_y != 0:
                squared_difference = abs((self.gps_speed_last_x - self.gps_robot_x)**2 - (self.gps_speed_last_y - self.gps_robot_y)**2)
                distance = math.sqrt(squared_difference)
            
            speed = (distance / ui_update_rate) * (1 / ui_update_rate)
            self.gstream_obj.speed_text = round(speed, 2)
            self.gps_speed_last_x, self.gps_speed_last_y = self.gps_robot_x, self.gps_robot_y
            self.last_update_time = time.time()
            #print(f"distance: {distance}, speed: {speed}")


    def mainloop(self):
        """
        Runs on seperate thread, doing all the gps logic
        which includes reading the log file and sending commands to the robot.
        Normal = 1000
        GPS driving = 1500  
        GPS save = 2000
        """

        while self.mainloop_running:
            self.arduino_line = self.read_arduino()
            
            self.write_arduino([int(self.to_motors*self.to_speed_limiter / 100), self.to_servo, self.to_brakes])
            self.update_camera_ui()

            # Stanley #### and not self.obstacle
            while not self.at_final_point and (self.arduino_line == 1500 or self.to_stanley_drive_toggle == 1):
                self.yaw = np.arctan2(self.gps_robot_y - self.gps_robot_last_y, self.gps_robot_x - self.gps_robot_last_x)

                self.update_camera_ui()
                if time.time() > self.previous_time + 0.2:
                    self.previous_time = time.time()
                    self.gps_robot_last_x, self.gps_robot_last_y = self.gps_robot_x, self.gps_robot_y

                limited_steering_angle, self.target_index, _, self.absolute_distance = stanley_controller(self.gps_robot_x, self.gps_robot_y, self.gps_x_col, self.gps_y_col, self.yaw, self.path_yaw, self.v, self.max_steering_control, self.k, self.k_s)
                limited_steering_angle = int(limited_steering_angle * (255 / self.max_steering_control))

                self.write_arduino([int((self.v*100) * self.to_speed_limiter / 100), -limited_steering_angle, 0])
                self.arduino_line = self.read_arduino()

        
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
    mainloop_thread = threading.Thread(target = m.mainloop)
    mainloop_thread.start()
    #####

    rclpy.spin(m)

    ##### Multithreading close ###
    #m.nano_running = False
    m.mainloop_running = False
    #encoder_read_thread.join()
    mainloop_thread.join()
    #####

    m.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
