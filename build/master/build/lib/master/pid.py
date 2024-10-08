class PIDController:
    def __init__(self):
        self.kp = 0.0  # Proportional gain
        self.ki = 0.0  # Integral gain
        self.kd = 0.0  # Derivative gain
        self.output_limits = 0  # Tuple of (min_output, max_output)
        self.integral = 0
        self.prev_error = 0.0

    def reset(self):
        """Reset the PID controller state."""
        self.integral = 0
        self.prev_error = 0.0

    def compute(self, crosstrack_error, dt, max_speed):
        proportional = crosstrack_error
        self.integral += crosstrack_error * dt
        derivative = (crosstrack_error - self.prev_error) / dt

        pid_output = (self.kp * proportional) + (self.ki * self.integral) + (self.kd * derivative)

        self.prev_error = crosstrack_error

        speed = max_speed - abs(pid_output)
        speed = max(max_speed*0.25 , min(speed, max_speed))

        return speed
