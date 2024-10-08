import pandas as pd
import random
#import time

class SavePath():
    def __init__(self):
        self.file_path: str = '/home/pimpmobile/ros2_ws/src/gps_publisher/gps_publisher/gps_log_data.csv'
        self.gps_df: pd.DataFrame = pd.read_csv(self.file_path) 
        self.threshold = 0.02

    def clear_csv(self) -> None:
        self.gps_df = self.gps_df.iloc[0:0]

    def write_to_csv(self, X:float, Y:float, Z:float) -> None:
        if len(self.gps_df.iloc[1:]) > 1:
            last_row = self.gps_df.iloc[-1]
            if abs(abs(last_row[0]) - abs(X)) < self.threshold and abs(abs(last_row[1]) - abs(Y)) < self.threshold:
                return
        new_row = {'X': X, 'Y': Y, 'Z': Z}
        self.gps_df.loc[len(self.gps_df)] = new_row
        self.gps_df.to_csv(self.file_path, index=False)

    def df_size(self) -> int:
        return (self.gps_df.shape[0] - 1)

def main():
    gps_log = SavePath()
    gps_log.clear_csv()
    gps_log.write_to_csv()

if __name__ == '__main__':
    main()