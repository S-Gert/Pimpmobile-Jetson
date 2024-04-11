import numpy as np

class Odometry:
    def __init__(self, ticks_per_revolution: int, wheel_distance_m: float, wheel_radius_m: float):
        self.ticks_per_revolution = ticks_per_revolution
        self.wheel_distance_m = wheel_distance_m
        self.wheel_radius_m = wheel_radius_m
        
        self.prev_tick_count_left = 0
        self.prev_tick_count_right = 0

        self.rotation_wheel_left = 0
        self.rotation_wheel_right = 0

        self.travel_left = 0
        self.travel_right = 0
        self.avg_travel = 0

        self.delta_ticks_left = 0
        self.delta_ticks_right = 0

        self.theta = 0.0  # orientation (in radians)
        
    
    def update(self, tick_count_left, tick_count_right):
        self.delta_ticks_left = tick_count_left - self.prev_tick_count_left
        self.delta_ticks_right = tick_count_right - self.prev_tick_count_right
        
        self.prev_tick_count_left = tick_count_left
        self.prev_tick_count_right = tick_count_right
        
        # Wheel rotation per tick in radians
        alpha = 2 * np.pi / self.ticks_per_revolution

        # wheel rotation (rad)
        self.rotation_wheel_left = alpha * self.delta_ticks_left
        self.rotation_wheel_right = alpha * self.delta_ticks_right 

        # Wheel travel
        self.travel_left += self.wheel_radius_m * self.rotation_wheel_left
        self.travel_right += self.wheel_radius_m * self.rotation_wheel_right
        
        # Average travel of both wheels in m
        self.avg_travel = round((self.travel_left + self.travel_right)/2, 2)

        # Rotation in rad
        if self.delta_ticks_left > self.delta_ticks_right:
            self.theta = (self.travel_right - self.travel_left)/self.wheel_distance_m
        else:
            self.theta = (self.travel_left - self.travel_right)/self.wheel_distance_m
        
    def get_travel(self):
        return self.avg_travel
    
    def get_orientation(self, unit: str = 'deg'):
        if unit == 'rad':
            return self.theta
        elif unit == 'deg':
            degrees = np.rad2deg(self.theta)
            degrees %= 360
            return round(degrees, 1)
        else:
            raise ValueError("not valid unit of angle")