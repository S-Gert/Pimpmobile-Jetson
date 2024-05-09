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
        
        self.sub_lidar = self.create_subscription(
            PoseWithCovariance,
            'gps_enu_pimp',
            self.gps_callback,
            10)

        self.sub_lidar 
        
        self.msg = String() #ros publish message type

        # Arduino mega serial
        #self.arduino_message = ""
        
        # Odometry variables
        self.nano_running = True
        self.arduino_nano_open = False
        self.encoders = '0, 0'
        self.odometry_object = odometry_object
        self.odometry_message = ""
        self.wheels_travel = 0
        self.orientation = 0

        # GPS variables
        self.gps_logic_running = True
        self.file_path: str = '/home/pimpmobile/ros2_ws/src/gps_publisher/gps_publisher/gps_log_data.csv'
        self.gps_df: pd.DataFrame = pd.read_csv(self.file_path)
        self.first_row_read = False
        self.itterate_start_row = 0

        self.gps_robot_x = 0
        self.gps_robot_y = 0
        self.gps_robot_z = 0
        
        self.gps_robot_last_x = 0
        self.gps_robot_last_y = 0
        self.gps_robot_last_z = 0

        self.gps_destination_x = 0
        self.gps_destination_y = 0
        self.gps_destination_z = 0
        self.gps_time = 0
        self.gps_orientation = 0

        self.at_destination_point = False
        self.path_complete = False

    def pub_callback(self):
        self.msg.data = self.odometry_message
        self.publisher_.publish(self.msg)

    def lidar_callback(self, msg):
        lid_arr = msg.data
        if lid_arr[1] == 1 or lid_arr[2] == 1 or lid_arr[3] == 1:
            self.obstacle = 1
        else:
            self.obstacle = 0
    
    def gps_callback(self, msg) -> None:    
        self.gps_robot_x = msg.pose.position.x
        self.gps_robot_y = msg.pose.position.y
        self.gps_robot_z = msg.pose.position.z
        #self.get_logger().info(f"{x = }, {y = }, {z = }")

    #Arduino mega serial:
    def write_to_arduino(self, arduino_message:str):
        # Message formating has to look like this: f"{motors}, {servo}, {brakes}"
        # Motors: -255 to 255
        # Servo: -255 to 255
        # Brakes 0 or 1
        arduino.write(bytes(str(arduino_message), 'utf-8'))
        
    def read_arduino(self):
        # Read arduino:
        arduino_line = arduino.readline().strip().decode()
        self.get_logger().info(arduino_line)

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

    def bearing_from_gps_coords(self, lat1, lat2, lon1, lon2) -> float:
        """
        returns bearing/orientation in degrees -180 to 180
        :param lat1: current x coordinate
        :param lat2: last x coordinate
        :param lon1: current y coordinate
        :param lon2: last y coordinate
        :return: float value in degrees
        """

        # convert to radians:
        g2r = math.pi/180

        lat1r = lat1 * g2r
        lat2r = lat2 * g2r
        lon1r = lon1 * g2r
        lon2r = lon2 * g2r
        dlonr = lon2r - lon1r

        y = math.sin(dlonr) * math.cos(lat2r)
        x = math.cos(lat1r) * math.sin(lat2r) - math.sin(lat1r) * math.cos(lat2r)*math.cos(dlonr)

        # compute bearning and convert back to degrees:
        bearing = math.atan2(y, x) / g2r
        return bearing

    def robot_movement_to_destination(self, robot_x: float, robot_y: float, dest_x: float, dest_y: float, orientation: float):
        # if robot_x < dest_x:
            # self.write_to_arduino("255, 0, 0")
        pass

    def read_gps_log_file(self) -> None:
        if self.gps_df.shape[0] > self.itterate_start_row: # checks if theres enough rows to even read
            row = list(self.gps_df.iloc[self.itterate_start_row])
            self.itterate_start_row += 1
            self.gps_destination_x = row[0]
            self.gps_destination_y = row[1]
            self.gps_destination_z = row[2]
            self.gps_time = row[3] # not used anywhere
            print(f'x: {self.gps_destination_x}, y: {self.gps_destination_y}, z: {self.gps_destination_z}, time: {self.gps_time}')
        else:
            print('No more rows to read')
            self.path_complete == True

    # GPS LOGIC RUNNING ON SEPERATE THREAD:
    def gps_logic(self) -> None:
        """
        returns nothing, runs on seperate thread, doing all the gps logic
        which includes reading the log file and sending commands to the robot.
        """
        while self.gps_logic_running:
            # read the first row once
            if self.first_row_read == False:
                self.read_gps_log_file()
                self.first_row_read = True
            #GPS logic:
            self.gps_orientation = self.bearing_from_gps_coords(self.gps_robot_x, self.gps_robot_last_x, self.gps_robot_y, self.gps_robot_last_y)

            # assume we are not at the destination
            self.at_destination_point = False

            # Destination check with tolerance:
            while not self.at_destination_point and not self.path_complete:
                distance_tolerance = 0.5
                at_x = False
                at_y = False
                if self.gps_destination_x - distance_tolerance < self.gps_robot_x < self.gps_destination_x + distance_tolerance:
                    at_x = True
                if self.gps_destination_y - distance_tolerance < self.gps_robot_y < self.gps_destination_y + distance_tolerance:
                    at_y = True

            # condition to read next row of coordinates:
            if at_x == True and at_y == True:
                self.read_gps_log_file()

            # if log file out of rows, set brakes and motor, servo to 0:
            if self.path_complete:
                self.write_to_arduino("0, 0, 1")
            elif not self.path_complete:
                self.robot_movement_to_destination(self.gps_robot_x, self.gps_robot_y, self.gps_destination_x, self.gps_destination_y, self.gps_orientation)


            self.gps_robot_last_x = self.gps_robot_x
            self.gps_robot_last_y = self.gps_robot_y

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

    ##### Multithreading gps log file reading and logic #####
    gps_logic_thread = threading.Thread(target = m.gps_logic)
    gps_logic_thread.start()
    #####

    rclpy.spin(m)

    ##### Multithreading close ###
    m.nano_running = False
    m.gps_logic_running = False
    encoder_read_thread.join()
    gps_logic_thread.join()
    #####

    m.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()