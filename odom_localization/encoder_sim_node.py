import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import random

class EncoderSimNode(Node):
    def __init__(self):
        super().__init__('encoder_sim_node')
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.pub = self.create_publisher(JointState, '/encoder/joint_states', 10)
        self.latest_msg = None

        self.noise_k = self.declare_parameter('noise_k', 0.0).value
        self.prev_true = None
        self.measured = None
        self.ticks_per_rev = self.declare_parameter('ticks_per_rev', 0).value
        self.step = 0.0
        if self.ticks_per_rev > 0:
            self.step = 2 * math.pi / self.ticks_per_rev                          # 空 1
    def joint_state_callback(self, msg):

        if self.prev_true is None:
            self.prev_true = list(msg.position)
            self.measured = list(msg.position)

        
        for i in range(len(msg.position)):
            dphi =  msg.position[i] - self.prev_true[i]                                     # 空 3、4
            sigma = self.noise_k * math.sqrt(abs(dphi))                  # 空 5、6
            self.measured[i] += dphi + random.gauss(0.0, sigma)    # 空 7、8
            self.prev_true[i] = msg.position[i]                               # 空 9

        out = JointState()
        out.header = msg.header                                             # 空 10
        out.name = msg.name                                                 # 空 11
        position = []
        for m in self.measured:
            if self.step > 0.0:
                m =  round(m / self.step) * self.step
            position.append(m)          # 在 if 外面：不量化时也要输出
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