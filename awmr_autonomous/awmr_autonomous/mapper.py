import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
import math

class AutoMapper(Node):
    def __init__(self):
        super().__init__('auto_mapper')

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_cb,
            10
        )

        self.pub = self.create_publisher(
            Int32MultiArray,
            '/awmr/cmd_motor',
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.front = float('inf')
        self.left  = float('inf')
        self.right = float('inf')
        
    def scan_cb(self, msg):
        front, left, right = [], [], []

        for i, r in enumerate(msg.ranges):
            if r < 0.05 or r > msg.range_max:
                continue
                
            angle = msg.angle_min + i * msg.angle_increment
            
            # FRONT: -15° to +15°
            if -0.26 < angle < 0.26:
                front.append(r)

            # LEFT: 30° to 90°
            elif 0.52 < angle < 1.57:
                left.append(r)

            # RIGHT: -90° to -30°
            elif -1.57 < angle < -0.52:
                right.append(r)

        self.front = min(front) if front else float('inf')
        self.left  = min(left)  if left  else float('inf')
        self.right = min(right) if right else float('inf')

    def control_loop(self):
        cmd = Int32MultiArray()
        cmd.data = [0, 0, 0]

        if self.front < 0.6:
            # Decide best direction
            if self.left > self.right:
                cmd.data = [-60, 60, 0]   # turn left
                self.get_logger().info('Obstacle ? turning LEFT')
            else:
                cmd.data = [60, -60, 0]   # turn right
                self.get_logger().info('Obstacle ? turning RIGHT')
        else:
            cmd.data = [70, 70, 0]       # forward

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = AutoMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
