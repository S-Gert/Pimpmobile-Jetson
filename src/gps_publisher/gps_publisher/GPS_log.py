import pandas as pd
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovariance

from rclpy import time
import serial
#import time


class GPS_log(Node):
    def __init__(self):
        super().__init__('gps_logger')
        self.sub_lidar = self.create_subscription(
            PoseWithCovariance,
            'gps_enu_pimp',
            self.enu_callback,
            10)

        self.file_path: str = '/home/pimpmobile/ros2_ws/src/gps_publisher/gps_publisher/gps_log_data.csv'
        self.gps_df: pd.DataFrame = pd.read_csv(self.file_path) 
        self.enu_callback

        self.gps_df = self.gps_df.iloc[0:0]

    def enu_callback(self, msg) -> None:    
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        t = msg.pose.orientation.x #time
        self.get_logger().info(f"{x = }, {y = }, {z = }, {t = }")
        self.write_to_csv(self.gps_df, x, y, z, t)

    def write_to_csv(self, df:pd.DataFrame, X:float, Y:float, Z:float, T:float) -> None:
        new_row = {'X': X, 'Y': Y, 'Z': Z, 'TIME': T}
        df.loc[len(df)] = new_row
        df.to_csv(self.file_path, index=False)

    def df_size(self, df:pd.DataFrame) -> int:
        return (df.shape[0] - 1)

def main(args=None):
    rclpy.init(args=args)
    gps_log = GPS_log()
    rclpy.spin(gps_log)
    gps_log.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()