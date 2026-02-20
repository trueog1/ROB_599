import numpy as np
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration
import time
import yaml
import os

class Controller(Node):
    def __init__(self, path):
        super().__init__('waypoint_controller')


        filename = 'src/hw3/hw3/waypoints3.yaml'
        script_dir = os.getcwd()
        self.file_path = os.path.join(script_dir, filename)

        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.0)

        # Get parameters
        self.waypoints = []
        self.load_waypoints_from_yaml()
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(Marker, 'waypoints', 10)

        # PID Error terms
        self.prev_error = 0.0
        self.integral = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.current_waypoint_index = 0

        self.x_odom = 0.0
        self.y_odom = 0.0
        self.theta = 0.0

        # Timer to run control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        self.marker_timer = self.create_timer(0.033, self.marker)

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def load_waypoints_from_yaml(self):
        with open(self.file_path, 'r') as f:
            data = yaml.safe_load(f)
    
        poses = []
        for wp in data['waypoints']:
            pose = PointStamped()
            pose.header.frame_id = wp['frame_id']
            #print(pose.header.frame_id)
            pose.point.x = wp['pose']['position']['x']
            pose.point.y = wp['pose']['position']['y']
            pose.point.z = wp['pose']['position']['z']
            self.waypoints.append((pose.point.x, pose.point.y))
            #print((pose.point.x, pose.point.y))
            
        print(self.waypoints)
        return poses
    
    def marker(self):
        goal = Marker()
        goal.header.frame_id = 'odom'
        goal.id = 0
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.type = Marker.SPHERE
        goal.action = Marker.ADD
        goal.pose.position.x = self.waypoints[self.current_waypoint_index][0]
        goal.pose.position.y = self.waypoints[self.current_waypoint_index][1]
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        goal.scale.x = 0.2
        goal.scale.y = 0.2
        goal.scale.z = 0.2
        goal.color.r = 0.0
        goal.color.g = 1.0
        goal.color.b = 0.0
        goal.color.a = 1.0

        self.goal_pub.publish(goal)
        self.get_logger().info('Publishing a list of poses to RViz')

    def control_loop(self):

        if not hasattr(self, 'stop_executed'):
            self.stop_executed = False  # Ensure flag exists

        if self.current_waypoint_index >= len(self.waypoints):
            if not self.stop_executed:  # Check if stop action has already been executed
                self.get_logger().info("Waypoints reached. Stopping robot.")
                self.stop_robot()
                self.get_logger().info("Successfully reached all waypoints.")
                self.stop_executed = True  # Set flag to prevent multiple stops
            return

        self.target_x, self.target_y = self.waypoints[self.current_waypoint_index]

        # Compute Euclidean distance to target
        distance = math.sqrt((self.target_x - self.robot_x) ** 2 + (self.target_y - self.robot_y) ** 2)

        if distance < 0.1:  # If close to waypoint, switch to next
            self.get_logger().info(f"Waypoint {self.current_waypoint_index + 1} reached: ({self.target_x}, {self.target_y})")
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                if not self.stop_executed:
                    self.get_logger().info("Waypoints reached. Stopping robot.")
                    self.stop_robot()
                    self.get_logger().info("Successfully reached all waypoints.")
                    self.stop_executed = True  # Ensure it runs only once
            return

        # Compute angle to waypoint
        target_angle = math.atan2(self.target_y - self.robot_y, self.target_x - self.robot_x)
        angle_error = target_angle - self.robot_yaw

        # Normalize angle error
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # PID Controller for angular velocity
        self.integral += angle_error * 0.1
        derivative = (angle_error - self.prev_error) / 0.1
        angular_vel = self.kp * angle_error + self.ki * self.integral + self.kd * derivative
        self.prev_error = angle_error

        # Proportional speed based on distance
        linear_vel = min(0.5, distance * 0.5)

        # Publish velocity
        twist = TwistStamped()
        twist.twist.linear.x = linear_vel
        twist.twist.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        twist = TwistStamped()
        twist.twist.linear.x = 0.0
        twist.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()