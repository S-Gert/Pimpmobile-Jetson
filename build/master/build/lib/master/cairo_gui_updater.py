from master.gstreamer_host import Gstream
import time
import math

class CairoUpdater():
    def __init__(self, Gstream_object):
        self.gstream_obj = Gstream_object

        self.gps_speed_last_x, self.gps_speed_last_y = 0, 0

        self.speed_last_update_time = 0
        self.to_reset_press_time = 0

    
    def update_speed(self, robot_x, robot_y):
        ui_update_rate = 0.5 # seconds

        if time.time() > self.speed_last_update_time + ui_update_rate:
            distance = 0
            if robot_x - self.gps_speed_last_x != 0 and robot_y - self.gps_speed_last_y != 0:
                squared_difference = abs((self.gps_speed_last_x - robot_x)**2 - (self.gps_speed_last_y - robot_y)**2)
                distance = math.sqrt(squared_difference)
            
            speed = (distance / ui_update_rate) * (1 / ui_update_rate)
            self.gstream_obj.speed_text = round(speed, 2)
            self.gps_speed_last_x, self.gps_speed_last_y = robot_x, robot_y
            self.speed_last_update_time = time.time()

    def update_stanley_reset(self, to_stanley_reset):
        if to_stanley_reset == 1:
            self.to_reset_press_time = time.time()
            self.gstream_obj.stanley_path_reset_state = 1
        elif time.time() > self.to_reset_press_time + 2:
            self.gstream_obj.stanley_path_reset_state = 0

    def update_speed_limiter(self, to_speed_limiter):
        self.gstream_obj.speed_limiter_text = to_speed_limiter

    def update_gps_status(self, robot_x, robot_y):
        self.gstream_obj.gps_status_text = f"X:{robot_x}, Y:{robot_y}"

    def update_stanley_params(self, k, v):
        self.gstream_obj.stanley_k_text = k
        self.gstream_obj.stanley_v_text = v

    def update_pid_values(self, final_speed, kp, ki, kd):
        self.gstream_obj.pid_values = [final_speed, kp, ki, kd]

    def update_gps_saving_toggle(self, gps_saving_toggle):
        if gps_saving_toggle == 1:
            self.gstream_obj.gps_path_saving_text = "SAVING PATH"
        else:
            self.gstream_obj.gps_path_saving_text = "Not Saving"

    def update_final_point_status(self, final_point_status):
        if final_point_status == 1:
            self.gstream_obj.stanley_at_final_point_text = "Path Complete"
        else:
            self.gstream_obj.stanley_at_final_point_text = "Not Complete"

    def update_stanley_running_status(self, stanley_drive_toggle):
        if stanley_drive_toggle == 1:
            self.gstream_obj.stanley_running_text = "Running"
        else:
            self.gstream_obj.stanley_running_text = "Not Running"

    def update_gear(self, to_handbrake_toggle, to_reverse_toggle):
        if to_handbrake_toggle == 1:
            self.gstream_obj.current_gear_text = "HB"
        elif to_reverse_toggle == 1:
            self.gstream_obj.current_gear_text = "R"
        else:
            self.gstream_obj.current_gear_text = "D"
