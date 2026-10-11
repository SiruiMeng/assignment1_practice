"""Drive a repeatable square route with timed velocity commands (on simulation time).

Each segment is sent from inside one node, so there is no start-up delay between
commands as there is with separate `ros2 topic pub` calls. The node exits when
the route is finished, after the robot has been stopped.
"""
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class DriveRouteNode(Node):
    def __init__(self):
        super().__init__('drive_route_node')
        side = self.declare_parameter('side', 1.0).value             # side length [m]
        v = self.declare_parameter('linear_speed', 0.1).value         # [m/s]
        w = self.declare_parameter('angular_speed', 0.3).value        # [rad/s]
        laps = self.declare_parameter('laps', 1).value
        self.start_delay = self.declare_parameter('start_delay', 5.0).value  # settle time [s]

        # Route = list of (linear speed, angular speed, duration); 1 s pauses let the robot stop
        pause = (0.0, 0.0, 1.0)
        straight = (v, 0.0, side / v)
        turn = (0.0, w, (math.pi / 2) / w)
        self.segments = [pause] + [straight, pause, turn, pause] * 4 * laps + [(0.0, 0.0, 2.0)]

        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.t0 = None
        self.index = 0
        self.segment_start = None
        self.done = False
        self.create_timer(0.02, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def send(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = v
        msg.twist.angular.z = w
        self.pub.publish(msg)

    def tick(self):
        t = self.now()
        if t == 0.0:            # /clock has not started yet
            return
        if self.t0 is None:
            self.t0 = t
            self.get_logger().info(f'Starting route in {self.start_delay:.1f} s')
        if t - self.t0 < self.start_delay:
            return
        if self.segment_start is None:
            self.segment_start = t

        # Advance to the next segment when the current one has elapsed
        while (self.index < len(self.segments)
               and t - self.segment_start >= self.segments[self.index][2]):
            self.segment_start += self.segments[self.index][2]
            self.index += 1

        if self.index >= len(self.segments):
            self.send(0.0, 0.0)
            self.get_logger().info('Route finished')
            self.done = True
            return
        v, w, _ = self.segments[self.index]
        self.send(v, w)


def main(args=None):
    rclpy.init(args=args)
    node = DriveRouteNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.send(0.0, 0.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
