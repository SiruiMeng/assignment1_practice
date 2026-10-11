import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import math


def quaternion_to_yaw(q):
    # Planar motion only needs yaw (rotation about z)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')
        self.create_subscription(Odometry, '/odometry/estimate', self.odometry_callback, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.ground_truth_callback, 10)
        self.latest_msg = None

        output_file_1 = self.declare_parameter('output_file_1', 'data.csv').get_parameter_value().string_value
        self.file = open(output_file_1, 'w', newline='', buffering=1)
        self.writer = csv.writer(self.file)
        self.writer.writerow(['stamp',
                              'est_x', 'est_y', 'est_theta',
                              'gt_x', 'gt_y', 'gt_theta'])
        self.get_logger().info(f'Logging to {output_file_1}')

    def ground_truth_callback(self, msg):
        # Skip until the first estimate has arrived
        if self.latest_msg is None:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        est = self.latest_msg.pose.pose
        gt = msg.pose.pose

        self.writer.writerow([
            stamp,
            est.position.x, est.position.y, quaternion_to_yaw(est.orientation),
            gt.position.x, gt.position.y, quaternion_to_yaw(gt.orientation),
        ])

    def odometry_callback(self, msg):
        self.latest_msg = msg

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
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
