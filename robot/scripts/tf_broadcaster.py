#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class ESP32TFRebroadcaster(Node):
    def __init__(self):
        super().__init__('esp32_tf_rebroadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            TransformStamped,
            '/tf',
            self.tf_callback,
            10)
    
    def tf_callback(self, msg):
        self.tf_broadcaster.sendTransform(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ESP32TFRebroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()