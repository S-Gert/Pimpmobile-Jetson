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
import atexit

from master.stanley_controller import Stanley
from master.pid import PIDController
from master.pathsave import SavePath
from master.stream_host import Gstream
from master.cairo_gui_updater import CairoUpdater

#arduino_nano_port = '/dev/ttyUSB0' # '/dev/ttyUSB1'
arduino_port = '/dev/ttyACM0'#'/dev/ttyACM1'
arduino = serial.Serial(port=arduino_port, baudrate=115200, timeout=1)
print("Opening port...")
time.sleep(2)

class Master(Node):
    def __init__(self):
        super().__init__('Pimpmaster')

        '''
        Initialize objects.
    
        '''
        self.stream_object = Gstream()
        self.cgui_updater = CairoUpdater(self.stream_object.get_cairo_overlay_object())
        self.stream_object.start_stream_in_thread()
        
        self.stanley_obj = Stanley()
        self.pid_obj = PIDController()
        self.pathsave_obj = SavePath()

        '''
        Subscribers
        '''
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
        
        '''
        VARIABLES
        '''

        self.mainloop_running = True

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
        self.to_stanley_k_addition = 0
        self.to_stanley_v_addition = 0

        # GPS variables
        self.file_path: str = '/home/pimpmobile/ros2_ws/src/gps_publisher/gps_publisher/gps_log_data.csv'
        self.gps_df: pd.DataFrame = pd.read_csv(self.file_path)
        self.gps_x_col = self.gps_df["X"].values 
        self.gps_y_col = self.gps_df["Y"].values
        self.path_yaw = self.stanley_obj.calculate_path_yaw(self.gps_x_col, self.gps_y_col)
        self.gps_logger_cleared = False 

        self.gps_robot_x = 0
        self.gps_robot_y = 0
        self.gps_robot_z = 0
    
        self.gps_robot_last_x = 0
        self.gps_robot_last_y = 0

        # Stanley variables
        self.at_final_point = False
        self.yaw = 0
        self.k = 0.3
        self.k_min, self.k_max = 0.3, 0.8
        self.v = 1.1 # 110 11.1ms
        self.v_min, self.v_max = 0.5, self.v
        self.k_s = 0
        self.target_index = 0
        self.absolute_distance = 0
        self.previous_time = 0
        self.max_steering_control = np.radians(45)

        # PID variables
        self.final_speed = 0
        self.declare_parameter('pimp_kp', 2)
        self.declare_parameter('pimp_ki', 0.03)
        self.declare_parameter('pimp_kd', 0.3)
        self.pid_obj.kp = self.get_parameter('pimp_kp').value
        self.pid_obj.ki = self.get_parameter('pimp_ki').value
        self.pid_obj.kd = self.get_parameter('pimp_kd').value
        self.last_dt = None



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
        self.to_save_path()

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

    def check_if_at_final_point(self):
        '''
        BROKEN TODO: fix it.
        '''
        if self.target_index > len(self.gps_x_col) - 9:
            if self.absolute_distance < 0.8:
                self.at_final_point = True
        self.at_final_point = False

    def update_path_yaw(self):
        self.gps_df: pd.DataFrame = pd.read_csv(self.file_path)
        self.gps_x_col = self.gps_df["X"].values 
        self.gps_y_col = self.gps_df["Y"].values
        self.path_yaw = self.stanley_obj.calculate_path_yaw(self.gps_x_col, self.gps_y_col)

    def to_save_path(self):
        if self.to_gps_saving_toggle == 1:
            if self.gps_logger_cleared == False:
                self.pathsave_obj.clear_csv()
                self.gps_logger_cleared = True
            self.pathsave_obj.write_to_csv(self.gps_robot_x, self.gps_robot_y, self.gps_robot_z)
        else:
            self.gps_logger_cleared = False

    def reset_stanley(self):
        self.at_final_point = False
        self.stanley_obj.prev_target_index = 0
        self.update_path_yaw()
        self.pid_obj.reset()

    def to_update_stanley_vk(self):
        if self.to_stanley_k_addition > 0:
            self.k += 0.01
            self.k = round(self.k, 3)
        elif self.to_stanley_k_addition < 0 and self.k > 0:
            self.k -= 0.01
            self.k = round(self.k, 3)

        if self.to_stanley_v_addition > 0:
            self.v += 0.01
            self.v = round(self.v, 3)
        elif self.to_stanley_v_addition < 0 and self.v > 0:
            self.v -= 0.01
            self.v = round(self.v, 3)

    def teleop_callback(self, msg):
        self.to_motors = msg.data[0]
        self.to_servo = msg.data[1]
        self.to_brakes = msg.data[2]
        self.to_reverse = msg.data[3]
        if self.to_reverse:
            self.to_motors = -self.to_motors
        
        self.to_speed_limiter = msg.data[4]
        self.to_handbrake_toggle = msg.data[5]
        if self.to_handbrake_toggle:
            self.to_motors = 0
            self.to_brakes = 1
        
        self.to_gps_saving_toggle = msg.data[6]
        self.to_stanley_reset = msg.data[7]
        if self.to_stanley_reset == 1:
            self.reset_stanley()
            
        self.to_stanley_drive_toggle = msg.data[8]
        self.to_stanley_k_addition = msg.data[9]
        self.to_stanley_v_addition = msg.data[10]
        self.to_update_stanley_vk()
        

    def update_camera_ui(self):
        self.cgui_updater.update_speed(self.gps_robot_x, self.gps_robot_y)
        self.cgui_updater.update_stanley_reset(self.to_stanley_reset)
        self.cgui_updater.update_speed_limiter(self.to_speed_limiter)
        self.cgui_updater.update_gps_status(self.gps_robot_x, self.gps_robot_y)
        self.cgui_updater.update_stanley_params(self.k, self.v)
        self.cgui_updater.update_pid_values(self.final_speed, self.pid_obj.kp, self.pid_obj.ki, self.pid_obj.kd)
        self.cgui_updater.update_gps_saving_toggle(self.to_gps_saving_toggle)
        self.cgui_updater.update_final_point_status(self.at_final_point)
        self.cgui_updater.update_stanley_running_status(self.to_stanley_drive_toggle)
        self.cgui_updater.update_gear(self.to_handbrake_toggle, self.to_reverse)

    def update_pid_params(self):
        self.pid_obj.kp = self.get_parameter('pimp_kp').value
        self.pid_obj.ki = self.get_parameter('pimp_ki').value
        self.pid_obj.kd = self.get_parameter('pimp_kd').value

    def map_range(self, value, value_min ,value_max ,mapped_min ,mapped_max):
        result = (value-value_min) * (mapped_max - mapped_min) / (value_max-value_min) + mapped_min
        return round(result, 2)

    def is_cone_obstacle(self):
        cone_detected = self.stream_object.area_limit_exceeded
        if cone_detected:
            self.obstacle = True
        else:
            self.obstacle = False

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

            while not self.at_final_point and (self.arduino_line == 1500 or self.to_stanley_drive_toggle == 1):
                self.update_camera_ui()
                
                #### STANLEY LOGIC, REPLACED WITH OPENCV LINE FOLLOWING. ####

                self.yaw = np.arctan2(self.gps_robot_y - self.gps_robot_last_y, self.gps_robot_x - self.gps_robot_last_x)

                if time.time() > self.previous_time + 0.2:
                    self.previous_time = time.time()
                    self.gps_robot_last_x, self.gps_robot_last_y = self.gps_robot_x, self.gps_robot_y

                limited_steering_angle, self.target_index, crosstrack_error, self.absolute_distance = self.stanley_obj.stanley_controller(
                    self.gps_robot_x, self.gps_robot_y, self.gps_x_col, self.gps_y_col, self.yaw,
                    self.path_yaw, self.v, self.max_steering_control, self.k, self.k_s)
                
                limited_steering_angle = int(limited_steering_angle * (255 / self.max_steering_control))

                # PID speed limiter
                if self.last_dt is not None:
                    dt = time.time() - self.last_dt
                else:
                    dt = 0.01
                self.last_dt = time.time()

                #self.update_pid_params()
                #pid_speed = self.pid_obj.compute(crosstrack_error, dt, self.v)

                # self.final_speed = int((pid_speed*100) * self.to_speed_limiter / 100)
                self.final_speed = int((self.v*100) * self.to_speed_limiter / 100)
                
                self.get_logger().info(f"{self.final_speed = }, P = {self.pid_obj.kp}, I = {self.pid_obj.ki}, D = {self.pid_obj.kd}")
                #print(f"PID speed output: {pid_speed}, final speed: {final_speed}, dt: {dt}, speed: {self.v}")
                #print(f"------------------------------------------------------")

                #self.k = self.map_range(pid_speed, self.v_min, self.v_max, self.k_max, self.k_min)

                self.is_cone_obstacle()
                if self.obstacle:
                    self.write_arduino([0, -limited_steering_angle, 1])
                else:
                    self.write_arduino([self.final_speed, -limited_steering_angle, self.to_brakes])
                
                '''
            while self.arduino_line == 1500:
                while self.obstacle:
                    self.write_arduino([0, 0, 1])
                
                servo_max_turn = 120 

                detected_line_center = self.stream_object.centerx
                centerx_range_min = 800
                centerx_range_max = 1000

                line_range_center = (centerx_range_min + centerx_range_max) / 2

                scaling_factor = servo_max_turn / (centerx_range_max - centerx_range_min)
                
                if detected_line_center > centerx_range_max:
                    distance_from_center = detected_line_center - line_range_center
                    #turn servo left
                    steering_angle = min(distance_from_center * scaling_factor, servo_max_turn)

                elif detected_line_center < centerx_range_min:
                    distance_from_center = line_range_center - detected_line_center
                    #turn servo right
                    steering_angle = -min(distance_from_center * scaling_factor, servo_max_turn)
                else:
                    steering_angle = 0
                
                self.final_speed = 70 * (self.to_speed_limiter / 100)
                print(f"speed: {self.final_speed}, turn: {steering_angle}")

                self.write_arduino([int(self.final_speed), int(steering_angle), self.to_brakes])
                self.arduino_line = self.read_arduino()'''

        
@atexit.register
def exit_handler() -> None:
    arduino.close()
    print('Closing arduino port...')

def main(args=None):
    rclpy.init(args=args)
    m = Master()

    ##### Multithreading gps log file reading and logic #####
    mainloop_thread = threading.Thread(target = m.mainloop)
    mainloop_thread.start()
    #####

    rclpy.spin(m)

    ##### Multithreading close ###
    m.mainloop_running = False
    mainloop_thread.join()
    #####

    m.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
