import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Odometry(Node):

    def __init__(self):
        super().__init__('Odometry')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.odometry_callback)
        self.i = 0
        self.encoder_max = 64
        self.encoder_max_length = 1.0 #m

    def odometry_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
    
    def calculate(self):



def main(args=None):
    rclpy.init(args=args)
    odom = Odometry()
    rclpy.spin(odom)
    odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
