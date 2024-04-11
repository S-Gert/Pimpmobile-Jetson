import math

class Odometry:
    def __init__(self, ticks_per_revolution: int, wheel_distance_cm: float, wheel_radius: float):
        self.ticks_per_revolution = ticks_per_revolution
        self.wheel_distance_cm = wheel_distance_cm
        self.wheel_radius_cm = wheel_radius
        
        self.x = 0.0  # x position
        self.y = 0.0  # y position
        self.theta = 0.0  # orientation (in radians)
        
        self.prev_tick_count_left = 0
        self.prev_tick_count_right = 0
    
    def update(self, tick_count_left, tick_count_right):
        delta_tick_left = tick_count_left - self.prev_tick_count_left
        delta_tick_right = tick_count_right - self.prev_tick_count_right
        
        self.prev_tick_count_left = tick_count_left
        self.prev_tick_count_right = tick_count_right
        
        distance_left = (2 * math.pi * self.wheel_radius_cm * delta_tick_left) / self.ticks_per_revolution
        distance_right = (2 * math.pi * self.wheel_radius_cm * delta_tick_right) / self.ticks_per_revolution
        
        #change in orientation (theta)
        delta_theta = (distance_right - distance_left) / self.wheel_distance_cm
        
        #Update orientation
        self.theta += delta_theta
        
        distance_travelled = (distance_left + distance_right) / 2.0
        
        self.x += distance_travelled * math.cos(self.theta)
        self.y += distance_travelled * math.sin(self.theta)
        
    def get_position(self):
        x_meters = round((self.x / 100), 2)
        y_meters = round((self.y / 100), 2)
        return x_meters, y_meters
    
    def get_orientation(self, unit: str = 'deg'):
        if unit == 'rad':
            return self.theta
        elif unit == 'deg':
            theta_deg = math.degrees(self.theta)
            theta_deg %= 360
            return theta_deg
        else:
            raise ValueError("not valid unit of angle")# Example usage: