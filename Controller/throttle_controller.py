import numpy as np

class ThrottlePIDController:
    def __init__(self):
        self.last_error = 0.0
        self.sum_error = 0.0
        self.Kp = 1  # Proportional gain
        self.Ki = 0.017  # Integral gain
        self.Kd = 1 # Derivative gain

    def update(self, desired_value, current_value):
        error = desired_value - current_value
        self.sum_error += error
        derivative = error - self.last_error
        self.last_error = error
        throttle = self.Kp * error + self.Ki * self.sum_error + self.Kd * derivative
        throttle = np.fmax(np.fmin(throttle, 0.5), 0)

        return throttle