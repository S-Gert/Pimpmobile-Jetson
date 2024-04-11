import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Int32MultiArray
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
        self.sub_lidar = self.create_subscription(
            Int32MultiArray,
            'lidar_pimp',
            self.lidar_callback,
            10)
        self.sub_gps = self.create_subscription(
            NavSatFix,
            'gps_data_pimp',
            self.gps_callback,
            10)

        self.sub_gps  # prevent unused variable warning
        self.sub_lidar 
        self.running = True
        self.arduino_nano_open = False
        self.encoders = '0, 0'
        self.odometry_object = odometry_object

    def lidar_callback(self, msg):
        lid_arr = msg.data
        if lid_arr[1] == 1 or lid_arr[2] == 1 or lid_arr[3] == 1:
            self.obstacle = 1
        else:
            self.obstacle = 0
    
    def gps_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.latitude)
    
    def read_arduino_nano_data(self):
        if self.arduino_nano_open == False:
            arduino_nano = serial.Serial(port=arduino_nano_port, baudrate=9600, timeout=.2)
            print("Opening arduino nano port...")
            time.sleep(3)
            self.arduino_nano_open = True
        while self.running:
            try:
                self.encoders = arduino_nano.readline().decode().strip()
                #self.get_logger().info(self.encoders)
                #print(self.encoders)
            except:
                print("Arduino Nano: cant decode data.")
            #self.odometry_stuff()

    ########### Odometry calculus #############

    def odometry_stuff(self):
        encoders_str_array = self.encoders.split(', ')
        encoder_ticks_left = int(encoders_str_array[1])
        encoder_ticks_right = int(encoders_str_array[0])
        
        self.odometry_object.update(encoder_ticks_left, encoder_ticks_right)
        x, y = self.odometry_object.get_position()
        orientation = self.odometry_object.get_orientation('deg')
        print(f"Position (m): {x = } {y = }")
        print(f"Orientation (deg): {orientation}")

    ########### GPS calculus

    # xy plot
    # Distance?
    


    ########### Master of puppets

    # Whole gang

@atexit.register
def exit_handler() -> None:
    arduino.close()
    print('Closing arduino port...')

def main(args=None):
    rclpy.init(args=args)
    odom_object = Odometry(64, 44, 31) # odometry object 64 ticks, 44 wheel distance, 31 wheel diameter (cm)
    m = Master(odom_object)

    ##### Multithreading arduino nano ###
    encoder_read_thread = threading.Thread(target = m.read_arduino_nano_data)
    encoder_read_thread.start()
    #####

    rclpy.spin(m)

    ##### Multithreading arduino nano ###
    m.running = False
    encoder_read_thread.join()
    #####

    m.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()