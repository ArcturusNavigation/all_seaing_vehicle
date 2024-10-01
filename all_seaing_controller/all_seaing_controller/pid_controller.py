class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.integral_min = -float("inf")
        self.integral_max = float("inf")
        self.reset()
    
    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
    
    def set_integral_min(self, val):
        self.integral_min = val

    def set_integral_max(self, val):
        self.integral_max = val
    
    def get_effort(self):
        return self.effort
    
    def update(self, feedback, dt):
        error = self.setpoint - feedback
        derivative = (error - self.prev_error) / dt
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_max), self.integral_min)
        self.effort = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error

    def reset(self):
        self.setpoint = 0
        self.prev_error = 0
        self.integral = 0
        self.effort = 0