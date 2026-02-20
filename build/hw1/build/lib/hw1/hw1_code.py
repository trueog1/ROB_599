#!/usr/bin/env python3

import numpy
import matplotlib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
import time

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Robot Mover Node Started')
        self.run_duration = Duration(seconds=10) # Set the duration to 10 seconds
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        if self.get_clock().now() - self.start_time <= Duration(seconds=4):
            msg.linear.x = 0.2  # Move forward at 0.2 m/s
            msg.angular.z = 0.0 # No turning
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg}"')

        else:
            msg = Twist()
            msg.linear.x = 0.2
            msg.angular.z = 0.5
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg}"')


def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()

    while rclpy.ok() and (node.get_clock().now() - node.start_time) < node.run_duration:
        rclpy.spin_once(node)
        node.get_logger().info('Node running in main loop...')
        time.sleep(0.1)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
