import numpy as np
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration
import time
import yaml

class WaypointController(Node):
    def __init__(self):
        super().__init__('waypoint_controller')

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.file_path = '/home/trueog/ros2_ws_ROB599/src/hw2/hw2/waypoint1.yaml'

        # Declare ROS 2 parameters
        self.declare_parameter('waypoint_1_x', 1.0)
        self.declare_parameter('waypoint_1_y', 2.0)
        self.declare_parameter('waypoint_2_x', 3.0)
        self.declare_parameter('waypoint_2_y', 2.0)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.0)

        # Get parameters
        #self.tranwaypoints = [(self.get_parameter('waypoint_1_x').value, self.get_parameter('waypoint_1_y').value),(self.get_parameter('waypoint_2_x').value, self.get_parameter('waypoint_2_y').value)]
        self.tranwaypoints = [(2.0, 3.0), (1.0, 4.0), (-2.0, 1.0), (-3.0, -5.0)]
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.att_pub = self.create_publisher(PoseStamped, 'attraction_vector', 1)
        self.rep_pub = self.create_publisher(PoseStamped, 'repulsion_vector', 1)
        self.fin_pub = self.create_publisher(PoseStamped, 'Final_Vector', 1)

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

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
        self.V_attraction = [0.0, 0.0]
        self.V_repulsion = [0.0,0.0]

        # Timer to run control loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    '''def load_waypoints_from_yaml(self):
        with open(self.file_path, 'r') as f:
            data = yaml.safe_load(f)
    
        poses = []
        for wp in data['waypoints']:
            pose = PointStamped()
            pose.header.frame_id = wp['frame_id']
            pose.point.x = wp['pose']['position']['x']
            pose.point.y = wp['pose']['position']['y']
            pose.point.z = wp['pose']['position']['z']
            self.waypoints.append(pose)

        return poses

    def transform_point(self):
        self.waypoints.header.stamp = self.get_clock().now().to_msg()
        self.tranwaypoints = []
        try:
            # 2. Transform the point to base_link
            for i in range(self.waypoints):
                transform = self.tf_buffer.lookup_transform("base_link", "map", rclpy.time.Time())
                transformed_point = tf2_geometry_msgs.do_transform_pose(self.waypoints[i], transform)
                #transformed_point = self.tf_buffer.transform(self.waypoints[i], 'base_link', timeout=Duration(seconds=0.1))
                self.get_logger().info(
                    f'Map Point: ({self.waypoints.point.x}, {self.waypoints.point.y}) -> '
                    f'Robot Point: ({transformed_point.point.x:.2f}, {transformed_point.point.y:.2f})'
                )
                self.tranwaypoints.append((transformed_point.point.x, transformed_point.point.y))
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'Could not transform: {e}')

        return self.tranwaypoints'''

    def control_loop(self):

        if not hasattr(self, 'stop_executed'):
            self.stop_executed = False  # Ensure flag exists

        if self.current_waypoint_index >= len(self.tranwaypoints):
            if not self.stop_executed:  # Check if stop action has already been executed
                self.get_logger().info("Waypoints reached. Stopping robot.")
                self.stop_robot()
                self.get_logger().info("Successfully reached all waypoints.")
                self.stop_executed = True  # Set flag to prevent multiple stops
            return


        self.target_x, self.target_y = self.tranwaypoints[self.current_waypoint_index]

        # Compute Euclidean distance to target
        distance = math.sqrt((self.target_x - self.robot_x) ** 2 + (self.target_y - self.robot_y) ** 2)


        if distance < 0.1:  # If close to waypoint, switch to next
            self.get_logger().info(f"Waypoint {self.current_waypoint_index + 1} reached: ({self.target_x}, {self.target_y})")
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.tranwaypoints):
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
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()