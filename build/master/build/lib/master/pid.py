class PIDController:
    def __init__(self):
        self.kp = 0.0  # Proportional gain
        self.ki = 0.0  # Integral gain
        self.kd = 0.0  # Derivative gain
        self.output_limits = 0  # Tuple of (min_output, max_output)
        self.integral = 0
        self.prev_error = None

    def reset(self):
        """Reset the PID controller state."""
        self.integral = 0
        self.prev_error = None

    def compute(self, error, dt, output_limits):
        """Compute the PID control output."""
        # Integral term calculation
        self.output_limits = (0.75 * output_limits, output_limits)

        self.integral += error * dt

        # Derivative term calculation
        derivative = 0
        if self.prev_error is not None and dt > 0:
            derivative = (error - self.prev_error) / dt

        # PID output calculation
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        print(f"{self.kp = }, {self.ki = }, {self.kd = }")

        # Apply output limits
        min_output, max_output = self.output_limits
        if min_output is not None:
            output = max(output, min_output)
        if max_output is not None:
            output = min(output, max_output)

        # Save current error for next derivative calculation
        self.prev_error = error

        return output
