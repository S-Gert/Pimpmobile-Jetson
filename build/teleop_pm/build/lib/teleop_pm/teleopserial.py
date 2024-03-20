import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import String, Int32MultiArray

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=.2)
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

    def write_data(self, x):
        #arduino.flushInput()
        x_str = str(x)
        arduino.write(bytes(x_str, 'utf-8'))
        self.get_logger().info(arduino.readline().strip().decode())
        #time.sleep(0.05)

    def lidar_callback(self, msg):
        lid_arr = msg.data
        if lid_arr[1] == 1 or lid_arr[2] == 1 or lid_arr[3] == 1:
            #self.get_logger().info("WALL!!!!!")
            self.obstacle = 1
        else:
            self.obstacle = 0

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
        #self.get_logger().info('I heard: "%s"' % steering_wheel)


def main(args=None):
    rclpy.init(args=args)

    teleop_serial = TeleopSerial()

    rclpy.spin(teleop_serial)

    teleop_serial.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
