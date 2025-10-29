import math

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.setpoint = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        self.effort = 0.0

        self.integral_min = -float("inf")
        self.integral_max = float("inf")
        self.effort_min = -float("inf")
        self.effort_max = float("inf")
        self.reset()
    
    def set_setpoint(self, setpoint):
        self.setpoint = setpoint

    def get_setpoint(self):
        return self.setpoint
    
    def set_integral_min(self, val):
        self.integral_min = val

    def set_integral_max(self, val):
        self.integral_max = val

    def set_effort_min(self, val):
        self.effort_min = val

    def set_effort_max(self, val):
        self.effort_max = val
    
    def get_effort(self):
        return max(min(self.effort, self.effort_max), self.effort_min)

    def is_done(self, feedback, threshold):
        return abs(self.setpoint - feedback) <= threshold
    
    def update(self, feedback, dt):
        if dt == 0:
            return
        error = self.setpoint - feedback
        derivative = (error - self.prev_error) / dt
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_max), self.integral_min)
        self.effort = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error

    def reset(self):
        self.setpoint = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        self.effort = 0.0

class CircularPID(PIDController):
    def is_done(self, feedback, threshold):
        feedback_mod = feedback % (2*math.pi)
        setpoint_mod = self.setpoint % (2*math.pi)
        diff_abs = abs(feedback_mod - setpoint_mod)
        return min(diff_abs, 2*math.pi - diff_abs) <= threshold

    def update(self, feedback, dt):
        if dt == 0:
            return
        error = (self.setpoint - feedback) % (2 * math.pi)
        if error > math.pi:
            error -= 2 * math.pi
        derivative = (error - self.prev_error) / dt
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_max), self.integral_min)
        self.effort = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error