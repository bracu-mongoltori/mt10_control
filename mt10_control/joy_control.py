#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')
        self.declare_parameter('linear_scale', 300.0)
        self.declare_parameter('angular_scale', 300.0)
        self.lin = 0.0
        self.ang = 0.0

        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value

        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 2)

        self.get_logger().info("JoyTeleop node started. Listening to /joy...")
        self.create_timer(0.4, self.joy_publish)
        
        self.prev_msg = None

    def joy_callback(self, msg: Joy):
        self.lin = msg.axes[1] * self.linear_scale  # Left stick vertical
        self.ang = msg.axes[3] * self.angular_scale  # Right stick horizontal
        

    def joy_publish(self):
        twist = Twist()
        twist.linear.x = self.lin
        twist.angular.z = self.ang
        
        
        self.twist_pub.publish(twist)
        self.get_logger().debug(f"Published Twist: linear.x={self.lin}, angular.z={self.ang}")

def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down JoyTeleop node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
