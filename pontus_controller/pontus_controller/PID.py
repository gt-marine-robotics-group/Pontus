from rclpy.time import Duration

class PID:
    def __init__(self,
                 kp: float, ki: float, kd: float, windup_max: float = None):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.prev_err = 0.0
        self.integral = 0.0

        # max magnitude to cap the integral term
        self.windup_max = windup_max

    def reset(self):
        self.prev_err = 0.0
        self.integral = 0.0

    def __call__(self, err: float, dt: Duration) -> float:
        """
        Return control value.

        Args:
        ----
        err (float): the error from the controller
        dt (Duration): the amount of time since the last time PID was calculated

        Return:
        ------
        float: the new control value

        """
        # Convert ROS duration to seconds
        dt = dt.nanoseconds * 1e-9

        # Check for valid dt
        if dt <= 0:
            print("invalid dt")
            return 0.0

        # compute integral of the error over time
        self.integral += err * dt

        # Clamp the integral term so it doesn't grow too large
        if self.windup_max is not None:
            self.integral = max(-self.windup_max, min(self.integral, self.windup_max))

        # compute derivative of the error
        d_err = (err - self.prev_err) / dt

        # compute the output feedback
        fb = (self.kp * err) + (self.ki * self.integral) + (self.kd * d_err)
        self.prev_err = err

        return fb