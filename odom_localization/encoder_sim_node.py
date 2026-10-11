"""Turn the ideal joint angles from Gazebo into simulated wheel-encoder readings.

Subscribes to /joint_states (true cumulative wheel angles) and publishes
/encoder/joint_states (measured cumulative wheel angles). Two error sources from
Lecture 2 can be switched on with parameters:

  noise_k        random error, sigma = noise_k * sqrt(|dphi|) per update (0 = off)
  ticks_per_rev  finite encoder resolution in ticks per revolution (0 = off)

With both parameters at 0 the node passes the readings through unchanged.
"""
import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class EncoderSimNode(Node):
    def __init__(self):
        super().__init__('encoder_sim_node')
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.pub = self.create_publisher(JointState, '/encoder/joint_states', 10)

        self.noise_k = self.declare_parameter('noise_k', 0.0).value
        self.prev_true = None   # true cumulative angles in the previous message [rad]
        self.measured = None    # measured cumulative angles kept by this node [rad]
        self.ticks_per_rev = self.declare_parameter('ticks_per_rev', 0).value
        self.step = 0.0         # angle of one encoder tick [rad]; 0 disables quantization
        if self.ticks_per_rev > 0:
            self.step = 2 * math.pi / self.ticks_per_rev

    def joint_state_callback(self, msg):
        # First message: start the measurement at the true angles (no error yet)
        if self.prev_true is None:
            self.prev_true = list(msg.position)
            self.measured = list(msg.position)

        # The noise is added to each increment and then accumulated, so the error
        # stays in the measured angle and drift grows with distance. Its variance is
        # proportional to |dphi|: a stationary wheel adds no noise and the total
        # error does not depend on the message rate.
        for i in range(len(msg.position)):
            dphi = msg.position[i] - self.prev_true[i]
            sigma = self.noise_k * math.sqrt(abs(dphi))
            self.measured[i] += dphi + random.gauss(0.0, sigma)
            self.prev_true[i] = msg.position[i]

        out = JointState()
        out.header = msg.header
        out.name = msg.name
        # Quantize only the published value. self.measured keeps full precision, so
        # the reading error stays within half a tick and does not accumulate.
        position = []
        for m in self.measured:
            if self.step > 0.0:
                m = round(m / self.step) * self.step
            position.append(m)
        out.position = position
        self.pub.publish(out)


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
