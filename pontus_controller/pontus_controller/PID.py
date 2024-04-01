class PID:

    def __init__(self, kp, ki, kd, windup_max=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.prev_err = 0.0
        self.integral = 0.0

        # max magnitude to cap the integral
        self.windup_max = windup_max

    def __call__(self, err, dt):

        dt = dt.nanoseconds * 1e-9 # Convert ROS duration to seconds

        # Check for valid dt
        if dt <= 0:
            print("invalid dt")
            return 0.0

        self.integral += err * dt # compute integral of the error over time

        # Clamp the integral term so it doesn't grow too large
        if self.windup_max != None:
            self.integral = max(-self.windup_max, min(self.integral, self.windup_max))

        d_err = (err - self.prev_err) / dt # compute derivative of the error

        u = (self.kp * err) + (self.ki * self.integral) + (self.kd * d_err) # compute the output

        self.prev_err = err

        return u
