import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray


class CmdVelToMotor(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_motor')

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_cb,
            10
        )

        self.pub = self.create_publisher(
            Int32MultiArray,
            '/awmr/cmd_motor',
            10
        )

        # ROBOT PARAMETERS
        self.wheel_base = 0.20     # meters
        self.max_pwm = 80

    def cmd_cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # differential drive
        vL = v - (w * self.wheel_base / 2.0)
        vR = v + (w * self.wheel_base / 2.0)

        # scale to PWM
        L = int(max(min(vL * 100, self.max_pwm), -self.max_pwm))
        R = int(max(min(vR * 100, self.max_pwm), -self.max_pwm))

        cmd = Int32MultiArray()
        cmd.data = [L, R, 0]

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = CmdVelToMotor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
