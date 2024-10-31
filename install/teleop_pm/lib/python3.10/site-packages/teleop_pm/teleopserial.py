import rclpy
import threading
from rclpy.node import Node
import serial
import time
from std_msgs.msg import String, Int32MultiArray

arduino_port = '/dev/ttyACM0'
arduino_nano_port = '/dev/ttyUSB0' # '/dev/ttyUSB1'

arduino = serial.Serial(port=arduino_port, baudrate=9600, timeout=.2)
print("Opening arduino port...")
time.sleep(3)

class TeleopSerial(Node):

    def __init__(self):
        super().__init__('teleop_serial')
        self.sub_lidar = self.create_subscription(
            Int32MultiArray,
            'lidar_pimp',
            self.lidar_callback,
            10)

        self.sub_wheel = self.create_subscription(
            String,
            'directions_to_yo_mommas_house',
            self.wheel_callback,
            9)
        
        self.sub_lidar  # prevent unused variable warning
        self.sub_wheel  # prevent unused variable warning
        self.arduino_commands = "0, 0, 0"
        self.obstacle = 0

        self.running = True
        self.arduino_nano_open = False
        self.encoders = '0, 0'

    # def read_arduino_data(self):
    #     self.get_logger().info(arduino.readline().strip().decode())

    def read_arduino_nano_data(self):
        if self.arduino_nano_open == False:
            arduino_nano = serial.Serial(port=arduino_nano_port, baudrate=9600, timeout=.2)
            print("Opening arduino nano port...")
            time.sleep(3)
            self.arduino_nano_open = True
        while self.running:
            self.encoders = arduino_nano.readline().decode().strip()
            #print(self.encoders)

    def write_data(self, x):
        x_str = str(x)
        arduino.write(bytes(x_str, 'utf-8'))
        arduino_line = arduino.readline().strip().decode()
        self.get_logger().info(arduino_line)
        #time.sleep(0.05)

    def lidar_callback(self, msg):
        lid_arr = msg.data
        if lid_arr[1] == 1 or lid_arr[2] == 1 or lid_arr[3] == 1:
            self.obstacle = 1
        else:
            self.obstacle = 0

# (wheel 255,gas -150,brake 0)
    def get_wheel_int_values(self, incoming_str):
        steering_wheel_arr = incoming_str.split(",")
        steering_wheel = int(steering_wheel_arr[0].strip("wheel"))
        gas_pedal = int(steering_wheel_arr[1].strip(" gas"))
        brake_pedal = int(steering_wheel_arr[2].strip(" brake"))
        reverse_gear_toggle = int(steering_wheel_arr[3].strip(" gear"))
        return steering_wheel, gas_pedal, brake_pedal, reverse_gear_toggle

    def wheel_callback(self, msg):
        sim_data = msg.data
        servo, motors, brakes, reverse_gear_toggle  = self.get_wheel_int_values(sim_data)
        if reverse_gear_toggle == 1:
            motors = motors*-1
        if self.obstacle == 1:
            brakes, motors = 1, 0
        self.get_logger().info(f"{motors}, {servo}, {brakes}")
        self.arduino_commands = f"{motors}, {servo}, {brakes}"
        self.write_data(self.arduino_commands) 

def main(args=None):
    rclpy.init(args=args)

    teleop_serial = TeleopSerial()

    encoder_read_thread = threading.Thread(target = teleop_serial.read_arduino_nano_data)
    encoder_read_thread.start()

    rclpy.spin(teleop_serial)
    teleop_serial.running = False
    encoder_read_thread.join()

    teleop_serial.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()