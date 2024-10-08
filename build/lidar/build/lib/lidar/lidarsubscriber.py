import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int32MultiArray


class Lidar(Node):

    def __init__(self):
        super().__init__('lidar_subscriber_publisher')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            100)

        self.publisher_ = self.create_publisher(Int32MultiArray, 'lidar_pimp', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.subscription  # prevent unused variable warning
        self.tolerance = 0.0872*12 # 5 kraadi * 24, 120 deg
        self.target_angle = 1.571 # (pi/2)
        self.obstacle_array = 0
        self.stopping_distance = 1.5
        # 180 -- 2.5222 per deg

    def obstacles(self, arr):
        all_sectors = []
        for sector in arr:
            count = 0
            for point in sector:
                if point < self.stopping_distance:
                    count += 1
            if count >= 3:
                all_sectors.append(True)
            else:
                all_sectors.append(False)
        return all_sectors
        #self.get_logger().info('obstacles: "%s"' % all_sectors)


    def listener_callback(self, msg):
        data = []
        start_index = int(round((self.target_angle - self.tolerance) / msg.angle_increment))
        end_index = int(round((self.target_angle + self.tolerance) / msg.angle_increment))
        for i in range(start_index, end_index):
            data.append(round(msg.ranges[i], 2))
        data = np.array_split(data, 5)
        self.obstacle_array = self.obstacles(data)

    def timer_callback(self):
        msg = Int32MultiArray()
        msg.data = self.obstacle_array
        self.publisher_.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    lidar = Lidar()
    rclpy.spin(lidar)
    lidar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
