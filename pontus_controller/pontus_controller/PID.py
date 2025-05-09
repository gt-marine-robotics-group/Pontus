from rclpy.time import Duration
from enum import Enum
from numpy import sign, pi


MASS = 34.02
VOLUME = 0.0405
DIAMETER = 0.2159
LENGTH = 0.6096
AREA = [pi * (DIAMETER / 2) ** 2, LENGTH * DIAMETER, LENGTH * DIAMETER]
# SURGE, SWAY, HEAVE, ROLL, PITCH, YAW
# Values taken from: https://phys.libretexts.org/Bookshelves/Classical_Mechanics/Classical_Mechanics_(Dourmashkin)/08%3A_Applications_of_Newtons_Second_Law/8.06%3A_Drag_Forces_in_Fluids  # noqa: E501
C = [0.47, 0.82, 0.82, 0.0, 0.0, 4.0]
WATER_DENSITY = 1000.0
GRAVITY = 9.8


class DegreeOfFreedom(Enum):
    SURGE = 0
    SWAY = 1
    HEAVE = 2
    ROLL = 3
    PITCH = 4
    YAW = 5


class PID:
    def __init__(self,
                 kp: float, ki: float, kd: float,
                 degree_of_freedom: DegreeOfFreedom = None, windup_max: float = None):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.prev_err = 0.0
        self.integral = 0.0

        # max magnitude to cap the integral
        self.windup_max = windup_max

        self.degree_of_freedom = degree_of_freedom

    def __call__(self, err: float, dt: Duration, desired_velocity: float = None) -> float:
        """
        Return control value.

        If a desired_velocity is provided, we will calculate the drag for a feed forward input.

        Args:
        ----
        err (float): the error from the controller
        dt (Duration): the amount of time since the last time PID was calculated
        desired_velocity (float): the desired_velocity we want to go

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

        acceleration_bouyancy = 0.0
        f_drag = 0.0

        # Calculate bouyancy force if controlling z
        if self.degree_of_freedom == DegreeOfFreedom.HEAVE:
            # F = pVg
            f_bouyancy = WATER_DENSITY * VOLUME * GRAVITY
            # F = mg
            f_gravity = MASS * GRAVITY
            f_net = f_bouyancy - f_gravity
            acceleration_bouyancy = f_net / MASS
        # Linear Drag: F = 1/2 CpAv^2
        # Approximated as a cylinder
        # if self.degree_of_freedom and 0 <= self.degree_of_freedom.value <= 2 and desired_velocity:
        #     f_drag_abs = 1/2 * C[self.degree_of_freedom.value] * WATER_DENSITY * AREA[self.degree_of_freedom.value] * desired_velocity ** 2  # noqa: E501
        #     f_drag = sign(desired_velocity) * f_drag_abs
        # Rotational Drag: TODO: Estimate this
        if self.degree_of_freedom and 3 <= self.degree_of_freedom.value and desired_velocity:
            f_drag_abs = C[self.degree_of_freedom.value] * desired_velocity ** 2
            f_drag = sign(desired_velocity) * f_drag_abs

        # feed forward
        ff = -acceleration_bouyancy + f_drag

        u = fb + ff
        # u = ff

        self.prev_err = err

        return u
