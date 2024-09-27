import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import time

class Stanley():

    def __init__(self) -> None:
        self.prev_target_index = 0
        self.prev_t = 0
        
    def calculate_path_yaw(self, x, y):
        # Calculate yaws for all the points in the saved file
        yaws = []
        for i in range(len(x) - 1):
            delta_x = x[i + 1] - x[i]
            delta_y = y[i + 1] - y[i]
            yaw = np.arctan2(delta_y, delta_x)
            yaws.append(yaw)
        yaws.append(yaws[-1])
        return np.array(yaws)

    def find_target_path_id(self, robot_x, robot_y, x, y, yaw):  
        # This is for finding the nearest point

        # Calculate position of the front axle
        fx = robot_x + 0.44 * np.cos(yaw)
        fy = robot_y + 0.44 * np.sin(yaw)

        # Calculate the difference between front axle and path points
        dx = fx - x
        dy = fy - y

        # Distance from the front axle to each point on the path
        d = np.hypot(dx, dy)

        # The shortest distance
        target_index = np.argmin(d)

        if target_index < self.prev_target_index:
            return self.prev_target_index, dx[self.prev_target_index], dy[self.prev_target_index], d[self.prev_target_index]
        
        self.prev_target_index = target_index

        return target_index, dx[target_index], dy[target_index], d[target_index]
        
    def calculate_yaw_term(self, target_index, yaw, path_yaw):
        # yaw error between robot and path
        # Return normalized angle between [-pi, pi]
        error = np.arctan2(np.sin(path_yaw[target_index] - yaw), np.cos(path_yaw[target_index] - yaw))
        return error

    def calculate_crosstrack_term(self, k, k_s, v, yaw, dx, dy, abs_distance):
        # Calculates cross-track steering error (lateral error)

        # Direction of robots front axle
        front_axle_vector = np.array([np.sin(yaw), -np.cos(yaw)])

        # Vector from robots front axle to nearest point
        nearest_path_vector = np.array([dx, dy])

        # Firstly finds the sign of the dot product
        # Sign value (1 or -1) indicates if the robot is going left or right from the path
        # Lastly we multiply the sign value with the absolute distance 
        crosstrack_error = np.sign(nearest_path_vector @ front_axle_vector) * abs_distance

        # Calculate the steering angle error (scaled by speed and coefficient)
        crosstrack_steering_error = np.arctan2((k * crosstrack_error), (v + k_s))

        return crosstrack_steering_error, crosstrack_error

    def stanley_controller(self, robot_x, robot_y, x, y, yaw, path_yaw, v, max_steering_control, k, k_s):
        """
        robot_x and robot_y = robot x and y coordinates
        dfX and dfY = saved path x and y waypoints
        yaw = robot heading direction
        v = speed
        steering_angle = angle of robots front wheels
        max_steering_control = maximum steering angle of the robot
        k = cross-track error coefficient
        k_s = speed damping
        dx and dy = x and y coordinate difference between robot front axle and nearest point
        abs_distance = absolute distance between robot front axle and nearest point
        """

        target_index, dx, dy, absolute_distance = self.find_target_path_id(robot_x, robot_y, x, y, yaw)
        yaw_error = self.calculate_yaw_term(target_index, yaw, path_yaw)
        crosstrack_steering_error, crosstrack_error = self.calculate_crosstrack_term(k, k_s, v, yaw, dx, dy, absolute_distance)

        desired_steering_angle = yaw_error + crosstrack_steering_error

        # Constrains steering angle to the vehicle limits
        if time.time() > (self.prev_t+0.5):
            self.prev_t = time.time()
            print(f"target indx: {target_index}, prev_index: {self.prev_target_index}, robot_x: {robot_x}, robot_y: {robot_y}, x: {dx}, y: {dy}")
            print(f"robot yaw: {yaw}, path yaw: {path_yaw[target_index]}")
            print(f"desired steering: {desired_steering_angle}, yaw error: {yaw_error}, crosst_steering_error: {crosstrack_steering_error}, crosstrack error: {crosstrack_error}")
            print("---------------------------------------------------------------------------------------------------------------")
        limited_steering_angle = np.clip(desired_steering_angle, -max_steering_control, max_steering_control)

        return limited_steering_angle, target_index, crosstrack_error, absolute_distance
        

# Example usage
if __name__ == "__main__":
    # Load GPS data
    gps_df = pd.read_csv("/home/markko/Downloads/gps_log_data.csv")
    
    # Load GPS data
    x = gps_df["X"].values
    y = gps_df["Y"].values

    # coef
    k = 1.0
    v = 2.0 # m/s
    k_s = 0.5

    max_steering_control = np.radians(45)  # Maximum steering control in radians

    # Initialize robot position and heading
    robot_gps = [5.0, 1.0]
    yaw = np.radians(79)  # Initial robot heading (yaw)

    path_yaw = calculate_path_yaw(x, y)

    target_index = 0

    while target_index < len(x) - 1:
        limited_steering_angle, target_index, crosstrack_error = stanley_controller(robot_gps[0], robot_gps[1], x, y, yaw, path_yaw, v, max_steering_control, k, k_s)
        
        # Update the robot's position and heading (simple simulation step)
        robot_gps[0] += v * np.cos(yaw) # Update x position
        robot_gps[1] += v * np.sin(yaw)  # Update y position
        yaw += v / 0.5 * np.tan(limited_steering_angle)  # Update yaw
