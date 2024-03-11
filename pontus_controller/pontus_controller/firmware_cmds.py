import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32MultiArray

class FirmwareCmdsNode(Node):

    def __init__(self):
        super().__init__('firmware_cmds')

        self.thruster_cmds = [0.0 for i in range(8)]

        self.thruster_pub = self.create_publisher(Float32MultiArray,
            'thrust_cmds', 10)

        self.callbacks = [
            self.thruster0_callback,
            self.thruster1_callback,
            self.thruster2_callback,
            self.thruster3_callback,
            self.thruster4_callback,
            self.thruster5_callback,
            self.thruster6_callback,
            self.thruster7_callback
        ]
        self.subs = []
        for i in range(8):
            self.subs.append(
                self.create_subscription(Float64, f'/pontus/thruster_{i}/cmd_thrust',
                    self.callbacks[i], 10)
            )

        self.create_timer(0.1, self.timer_callback)
    
    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [cmd / 52.0 for cmd in self.thruster_cmds]
        self.thruster_pub.publish(msg)
    
    def thruster0_callback(self, msg: Float64):
        self.thruster_cmds[0] = msg.data

    def thruster1_callback(self, msg: Float64):
        self.thruster_cmds[1] = msg.data

    def thruster2_callback(self, msg: Float64):
        self.thruster_cmds[2] = msg.data

    def thruster3_callback(self, msg: Float64):
        self.thruster_cmds[3] = msg.data

    def thruster4_callback(self, msg: Float64):
        self.thruster_cmds[4] = msg.data

    def thruster5_callback(self, msg: Float64):
        self.thruster_cmds[5] = msg.data

    def thruster6_callback(self, msg: Float64):
        self.thruster_cmds[6] = msg.data
    
    def thruster7_callback(self, msg: Float64):
        self.thruster_cmds[7] = msg.data


def main(args=None):
    rclpy.init(args=args)

    node = FirmwareCmdsNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()