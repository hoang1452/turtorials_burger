class PIDController:
    def __init__(self, Kp, Ki, Kd, output_min, output_max):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0
        self.output_min = output_min
        self.output_max = output_max

    def compute(self,error,dt):

        # Proportional term
        P = self.Kp * error

        # Integral term
        self.integral += error * dt
        I = self.Ki * self.integral

        # Derivative term
        derivative = (error - self.last_error) / dt
        D = self.Kd * derivative

        # PID output
        output = P + I + D

        # Apply output limits
        output = max(self.output_min, min(output, self.output_max))

        # Update last error for next iteration
        self.last_error = error

        return output
    