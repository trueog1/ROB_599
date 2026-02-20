import numpy
import matplotlib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Robot Mover Node Started')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.2  # Move forward at 0.2 m/s
        msg.angular.z = 0.0 # No turning
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg}"')

def main(args=None):
    rclpy.init(args=args)
    mover_node = RobotMover()
    rclpy.spin(mover_node) # Keep the node running
    mover_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
