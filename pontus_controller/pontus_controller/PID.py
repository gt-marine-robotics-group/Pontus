class PID:

    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.prev_err = 0.0
        self.integral = 0.0

    def __call__(self, err, dt):

      dt = dt.nanoseconds * 1e-9 # Convert ROS duration to seconds

      # Check for valid dt
      if dt <= 0:
          print("invalid dt")
          return 0.0

      self.integral += err * dt # compute integral of the error over time
      d_err = (err - self.prev_err) / dt # compute derivative of the error

      u = (self.kp * err) + (self.ki * self.integral) + (self.kd * d_err) # compute the output

      self.prev_err = err

      return u
