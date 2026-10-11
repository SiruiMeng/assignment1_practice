import math


def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def wheel_to_body(ds_left, ds_right, wheel_separation):
    ds = (ds_left + ds_right) / 2
    dtheta = (ds_right - ds_left) / wheel_separation
    return ds, dtheta


def update_pose(x, y, theta, ds, dtheta):
    mid_theta = theta + dtheta / 2.0
    x_new = x + ds*math.cos(mid_theta)
    y_new = y + ds*math.sin(mid_theta)

    theta_new = theta + dtheta

    theta_new = wrap_angle(theta_new)

    return x_new, y_new, theta_new
