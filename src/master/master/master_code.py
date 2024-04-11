import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Int32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy 

import pandas as pd
import threading
import serial
import time
from master.odometry import Odometry
import atexit

arduino_nano_port = '/dev/ttyUSB0' # '/dev/ttyUSB1'
arduino_port = '/dev/ttyACM0'

arduino = serial.Serial(port=arduino_port, baudrate=9600, timeout=.2)
print("Opening arduino port...")
time.sleep(3)

class Master(Node):

    def __init__(self, odometry_object):
        super().__init__('Master_Code')

        # qos_profile = QoSProfile(
        #     reliability = ReliabilityPolicy.RELIABLE,
        #     history = HistoryPolicy.KEEP_ALL,
        #     depth = 1
        # )

        # qos_profile = QoSProfile(
        #     reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, 
        #     history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        #     depth = 1
        # )

        self.publisher_ = self.create_publisher(String, 'pimp_odo', 1)
        time_period = 0.1
        self.timer = self.create_timer(time_period, self.pub_callback)

        self.sub_lidar = self.create_subscription(
            Int32MultiArray,
            'lidar_pimp',
            self.lidar_callback,
            10)

        self.sub_lidar 
        self.nano_running = True
        self.gps_log_running = True
        self.arduino_nano_open = False
        self.encoders = '0, 0'
        self.odometry_object = odometry_object
        
        self.msg = String() #ros publish message type
        self.odometry_message = ""

        self.wheels_travel = 0
        self.orientation = 0

        self.file_path: str = '/home/pimpmobile/ros2_ws/src/gps_publisher/gps_publisher/gps_log_data.csv'
        self.gps_df: pd.DataFrame = pd.read_csv(self.file_path)
        self.itterate_start_row = 0
        self.gps_x = 0
        self.gps_y = 0
        self.gps_z = 0
        self.gps_time = 0

    def pub_callback(self):
        self.msg.data = self.odometry_message
        self.publisher_.publish(self.msg)

    def lidar_callback(self, msg):
        lid_arr = msg.data
        if lid_arr[1] == 1 or lid_arr[2] == 1 or lid_arr[3] == 1:
            self.obstacle = 1
        else:
            self.obstacle = 0
    
    def read_arduino_nano_data(self):
        if self.arduino_nano_open == False:
            try:
                arduino_nano = serial.Serial(port=arduino_nano_port, baudrate=9600, timeout=.2)
                print("Opening arduino nano port...")
                time.sleep(3)
                self.arduino_nano_open = True
            except Exception:
                print("Couldn't open nano port")
        while self.nano_running:
            try:
                self.encoders = arduino_nano.readline().decode().strip()
                #self.get_logger().info(self.encoders)
                #print(self.encoders)
                self.odometry_stuff()
            except:
                print("Arduino Nano: cant decode data.")

    ########### Odometry calculus #############

    def odometry_stuff(self):
        encoders_str_array = self.encoders.split(', ')
        encoder_ticks_left = int(encoders_str_array[1])
        encoder_ticks_right = int(encoders_str_array[0])
        
        self.odometry_object.update(encoder_ticks_left, encoder_ticks_right)
        self.wheels_travel = self.odometry_object.get_travel()
        self.orientation = self.odometry_object.get_orientation('deg')

        self.odometry_message = f"{self.wheels_travel = }, {self.orientation = }"
        
        #print(f"Distance travelled (m): {self.wheels_travel}")
        #print(f"Orientation (deg): {self.orientation}")

    ########### GPS calculus ##########

    def gps_log_file_read(self):
        while self.gps_log_running:
            if self.gps_df.shape[0] > self.itterate_start_row: # checks if theres enough rows to even read
                row = list(self.gps_df.iloc[self.itterate_start_row])
                self.itterate_start_row += 1
                self.gps_x = row[0]
                self.gps_y = row[1]
                self.gps_z = row[2]
                self.gps_time = row[3]
                print(f'x: {self.gps_x}, y: {self.gps_y}, z: {self.gps_z}, time: {self.gps_time}')
            else:
                print('No more rows to read')
            time.sleep(0.5)

@atexit.register
def exit_handler() -> None:
    arduino.close()
    print('Closing arduino port...')

def main(args=None):
    rclpy.init(args=args)
    odom_object = Odometry(64, 0.44, 0.155) # odometry object 64 ticks, 44 wheel distance, 15.5 wheel radius (m)
    m = Master(odom_object)


    ##### Multithreading arduino nano ###
    encoder_read_thread = threading.Thread(target = m.read_arduino_nano_data)
    encoder_read_thread.start()

    ##### Multithreading gps log file reading #####
    gps_log_thread = threading.Thread(target = m.gps_log_file_read)
    gps_log_thread.start()
    #####

    rclpy.spin(m)

    ##### Multithreading arduino nano ###
    m.nano_running = False
    m.gps_log_running = False
    encoder_read_thread.join()
    gps_log_thread.join()
    #####

    m.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()