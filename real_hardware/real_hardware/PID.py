import math as m

# This class implements a PID controller. It allows also the control of an angle.
class PIDController:
    def __init__(self, angle: bool = False, kp: float = 20, ki: float = 0.1, kd: float = 0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.angle = angle

        self.M_PI = m.pi
        self.prev_error = 0
        self.integral = 0

    def calc(self, state: float, target: float, dt: float, bias: float = 0.0):
        error = target - state
        if self.angle:
            while(error < -self.M_PI):
                error += 2 * self.M_PI
            while(error > self.M_PI):
                error -= 2 * self.M_PI
        self.integral += (error * dt)
        derivative = (error - self.prev_error)/(dt)
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative + bias
