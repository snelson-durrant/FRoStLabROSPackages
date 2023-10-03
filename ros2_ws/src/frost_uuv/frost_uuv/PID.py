class PIController:
    def __init__(self, kp, ki, min_output, max_output):
        # Constants
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain

        # Output limits
        self.min_output = min_output
        self.max_output = max_output

        # State variables
        self.integral = 0.0

    def compute(self, setpoint, current_value):
        # Calculate the error
        error = setpoint - current_value

        # Calculate the integral term
        self.integral += error

        # Apply limits to the integral term to prevent windup
        if self.integral > self.max_output:
            self.integral = self.max_output
        elif self.integral < self.min_output:
            self.integral = self.min_output

        # Calculate the control signal (output)
        output = self.kp * error + self.ki * self.integral

        # Apply output limits
        if output > self.max_output:
            output = self.max_output
        elif output < self.min_output:
            output = self.min_output

        return output