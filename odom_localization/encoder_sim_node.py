import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class EncoderSimNode(Node):
    def __init__(self):
        super().__init__('encoder_sim_node')
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.pub = self.create_publisher(JointState, '/encoder/joint_states', 10)
        self.latest_msg = None

    def joint_state_callback(self, msg):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderSimNode()
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