"""Estimate the robot pose by dead reckoning from wheel-encoder readings only.

Subscribes to the encoder JointState topic and publishes the estimated pose on
/odometry/estimate. It never reads /odom, /tf or the simulator ground truth.
"""
import math

from nav_msgs.msg import Odometry
from odom_localization.odometry_math import update_pose, wheel_to_body
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Parameters can be overridden at runtime with --ros-args -p name:=value
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.033).value
        self.wheel_separation = self.declare_parameter('wheel_separation', 0.160).value
        self.left_joint = self.declare_parameter('left_joint', 'wheel_left_joint').value
        self.right_joint = self.declare_parameter('right_joint', 'wheel_right_joint').value
        joint_topic = self.declare_parameter('joint_topic', '/encoder/joint_states').value

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_left_pos = None
        self.prev_right_pos = None

        self.odom_pub = self.create_publisher(Odometry, '/odometry/estimate', 10)
        self.create_subscription(JointState, joint_topic, self.joint_state_callback, 10)

    def joint_state_callback(self, msg):
        try:
            left_pos = msg.position[msg.name.index(self.left_joint)]
            right_pos = msg.position[msg.name.index(self.right_joint)]
        except ValueError:
            self.get_logger().warn(
                f'Joints {self.left_joint}/{self.right_joint} not in {list(msg.name)}',
                throttle_duration_sec=5.0)
            return

        # The first message has no previous position: store it, do not integrate
        if self.prev_left_pos is not None and self.prev_right_pos is not None:

            # wheel angle increment [rad]
            dphi_left = left_pos - self.prev_left_pos
            dphi_right = right_pos - self.prev_right_pos

            # wheel travelled distance [m]
            ds_left = self.wheel_radius * dphi_left
            ds_right = self.wheel_radius * dphi_right

            # wheel motion -> robot body motion
            ds, dtheta = wheel_to_body(ds_left, ds_right, self.wheel_separation)

            # update robot pose
            self.x, self.y, self.theta = update_pose(
                self.x, self.y, self.theta, ds, dtheta)

            # create odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = msg.header.stamp
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'

            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y

            # yaw -> quaternion
            odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
            odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

            self.get_logger().info(
                f'Publishing odometry: '
                f'x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}',
                throttle_duration_sec=1.0)

            self.odom_pub.publish(odom_msg)

        # save current wheel positions for next callback
        self.prev_left_pos = left_pos
        self.prev_right_pos = right_pos


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
