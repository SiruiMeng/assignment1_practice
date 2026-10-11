"""Differential-drive odometry equations from Lecture 2 (pure functions, no ROS)."""
import math


def wrap_angle(angle):
    """Wrap an angle in radians to the interval [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def wheel_to_body(ds_left, ds_right, wheel_separation):
    """Convert left/right wheel travel [m] into robot travel ds [m] and turn dtheta [rad]."""
    ds = (ds_left + ds_right) / 2
    dtheta = (ds_right - ds_left) / wheel_separation
    return ds, dtheta


def update_pose(x, y, theta, ds, dtheta):
    """Advance the pose (x [m], y [m], theta [rad]) by one odometry step.

    The step is taken along the heading at the middle of the step, theta + dtheta/2.
    """
    mid_theta = theta + dtheta / 2.0
    x_new = x + ds * math.cos(mid_theta)
    y_new = y + ds * math.sin(mid_theta)
    theta_new = wrap_angle(theta + dtheta)
    return x_new, y_new, theta_new
