import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PointStamped, PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_geometry_msgs
import math
from rclpy.duration import Duration
import time
import yaml
import os

class PotentialField(Node):

    def __init__(self):
        super().__init__('potential_field_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        filename = 'src/hw2/hw2/waypoint1.yaml'
        script_dir = os.getcwd()
        self.file_path = os.path.join(script_dir, filename)

        #self.waypoints = [(2.0, 3.0), (1.0, 4.0), (-2.0, 1.0), (-3.0, -5.0), (0.0, -2.0)]
        self.waypoints = []
        self.load_waypoints_from_yaml()
        #self.goal_x = float(x_goal)
        #self.goal_y = float(y_goal)

        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 1)
        self.att_pub = self.create_publisher(PoseStamped, 'attraction_vector', 1)
        self.rep_pub = self.create_publisher(PoseStamped, 'repulsion_vector', 1)
        self.fin_pub = self.create_publisher(PoseStamped, 'Final_Vector', 1)
        self.goal_pub = self.create_publisher(Marker, 'waypoints', 10)

        self.x_odom = 0.0
        self.y_odom = 0.0
        self.theta = 0.0
        self.obstacles = []
        self.prev_error = 0.0
        self.integral = 0.0
        self.V_attraction = [0.0, 0.0]
        self.V_repulsion = [0.0,0.0]
        self.current_waypoint_index = 0
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.0)
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        self.timer = self.create_timer(0.1, self.controller)
        self.marker_timer = self.create_timer(0.033, self.marker)
        

    def load_waypoints_from_yaml(self):
        with open(self.file_path, 'r') as f:
            data = yaml.safe_load(f)
    
        poses = []
        for wp in data['waypoints']:
            pose = PointStamped()
            pose.header.frame_id = wp['frame_id']
            pose.point.x = wp['pose']['position']['x']
            pose.point.y = wp['pose']['position']['y']
            pose.point.z = wp['pose']['position']['z']
            self.waypoints.append((pose.point.x, pose.point.y))

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

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q
    
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def publish_vector(self, x, y):
        vector = PoseStamped()
        vector.header.frame_id = '/odom'
        vector.header.stamp = self.get_clock().now().to_msg()
        vector.pose.position.x = self.x_odom
        vector.pose.position.y = self.y_odom
        vector.pose.position.z = 0.0

        angle = math.atan2(y, x) if x >= 0 else math.pi + math.atan2(y, x)
        q = self.quaternion_from_euler(0, 0, angle)
        vector.pose.orientation.x = q[3]
        vector.pose.orientation.y = q[2]
        vector.pose.orientation.z = q[1]
        vector.pose.orientation.w = q[0]

        return vector
    
    def goal_mag(self, x):
        x2 = np.power(x,2)
        x4 = np.power(x,4)
        return -35*np.exp(-x2/(1e-2))+ 7*np.exp(-x4/(4e4)) + 30*np.exp(-x2/4)
    
    def ob_mag(self, x):
        x2 = np.power(x,2)
        x4 = np.power(x,4)
        return 100*np.exp(-x2/(1e-2))+ 0.1*np.exp(-x4/(4e4)) + 0.14*np.exp(-x2/.7)
    
    def apf(self):
        target_force = np.array([0.0,0.0])
        theta = math.atan2(self.y_odom, self.x_odom)
        target_distance = np.linalg.norm([np.array([self.x_odom, self.y_odom])])
        target_mag_att = self.goal_mag(target_distance)
        target_force = target_mag_att * np.array([math.cos(theta), math.sin(theta)])

        ob_force = np.array([0.0,0.0])
        count = 0
        for i in self.obstacles:
            theta_o = math.atan2(i[1], i[0])
            ob_vector = np.array([i[0],i[1]])
            distance = np.linalg.norm(ob_vector)
            ob_force += self.ob_mag(distance) * np.array([-math.cos(theta_o), -math.sin(theta_o)])
            if distance < 5.0:
                count +=1


        total_force = target_force + ob_force
        return total_force/4

    def compute_attraction(self, x_goal, y_goal):
        self.get_logger().info(f"GOAL | x: {x_goal} | y: {y_goal}")

        dx = x_goal - self.x_odom
        dy = y_goal - self.y_odom
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 1e-3:
            self.get_logger().warn("Goal is too close to the current position; force set to zero.")
            self.V_attraction = [0.0, 0.0]
            return

        Q_attraction = 2
        F_attraction = Q_attraction * distance
        self.V_attraction = [F_attraction * dx / distance, F_attraction * dy / distance]

        self.get_logger().info(f"Attraction Force | x: {self.V_attraction[0]} | y: {self.V_attraction[1]}")

        attraction = self.publish_vector(self.V_attraction[0], self.V_attraction[1])
        self.att_pub.publish(attraction)

    def odom_callback(self, msg):
        self.x_odom = msg.pose.pose.position.x
        self.y_odom = msg.pose.pose.position.y

        quaternion = msg.pose.pose.orientation
        roll, pitch, self.theta = self.euler_from_quaternion(quaternion)

        #self.compute_attraction(self.target_x, self.target_y)

    def scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        scan = msg.ranges
        num_readings = len(msg.ranges)

        for i in range(num_readings):
            distance = scan[i]
            if distance < msg.range_max:
                angle = angle_min + i * angle_increment
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                self.obstacles.append((x,y))

    '''def apf_field_move(self):
        force = self.apf()
        force_angle = math.atan2(force[1], force[0])
        distance = math.sqrt(force[0]**2 + force[1]**2)
        self.move.twist.linear.x = 0.8*min(0.5, distance * 0.5) + 0.2*self.move.twist.linear.x
        self.move.twist.angular.z = 0.9*force_angle + 0.1*self.move.twist.angular.z
        self.cmd_pub.publish(self.move)
        self.get_logger().info("Moving along field")'''

    '''def controller(self):
        self.marker()
        self.target_x = self.waypoints[self.current_waypoint_index][0]
        self.target_y = self.waypoints[self.current_waypoint_index][1]
        move = TwistStamped()
        angle_t = math.atan2((self.target_y - self.y_odom), (self.target_x - self.x_odom))

        if self.obstacles:
            for ob in self.obstacles:
                angle = math.atan2(ob[1], ob[0])
                if abs(angle - angle_t) < 0.2 and math.sqrt(ob[0]**2 + ob[1]**2):'''

    def controller(self):

        if not hasattr(self, 'stop_executed'):
            self.stop_executed = False  # Ensure flag exists

        if self.current_waypoint_index >= len(self.waypoints):
            if not self.stop_executed:  # Check if stop action has already been executed
                self.get_logger().info("Waypoints reached. Stopping robot.")
                self.stop_robot()
                self.get_logger().info("Successfully reached all waypoints.")
                self.stop_executed = True  # Set flag to prevent multiple stops
            return
        
        direction = TwistStamped()
        
        goal_tolerance = 0.1
        tolerance = 0.1
        rotation_gain = 0.75
        max_rotation_speed = 0.5
        forward_speed = 0.2

        #total_V_rep_x = sum(self.V_repulsion[:][0])
        #total_V_rep_y = sum(self.V_repulsion[:][1])
        self.marker()
        self.target_x = self.waypoints[self.current_waypoint_index][0]
        self.target_y = self.waypoints[self.current_waypoint_index][1]
        '''self.compute_attraction(self.target_x, self.target_y)

        x_final = self.V_attraction[0] + self.V_repulsion[0]
        y_final = self.V_attraction[1] + self.V_repulsion[1]'''

        #distance = math.sqrt(x_final**2 + y_final**2)
        distance = math.sqrt((self.target_x - self.x_odom) ** 2 + (self.target_y - self.y_odom) ** 2)

        if distance < goal_tolerance:  # If close to waypoint, switch to next
            self.get_logger().info(f"Waypoint {self.current_waypoint_index + 1} reached: ({self.target_x}, {self.target_y})")
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                if not self.stop_executed:
                    self.get_logger().info("Waypoints reached. Stopping robot.")
                    self.stop_robot()
                    self.get_logger().info("Successfully reached all waypoints.")
                    self.stop_executed = True  # Ensure it runs only once
            return
        
        '''if distance < goal_tolerance:
            direction.linear.x = 0.0
            direction.angular.z = 0.0
            self.get_logger().info("Target reached!")
            self.cmd_pub.publish(direction)
            self.stop_robot()
            return 0'''

        '''finalvector = self.publish_vector(x_final, y_final)
        self.fin_pub.publish(finalvector)'''
        force = self.apf()
        force_angle = math.atan2(force[1], force[0])
        delta = self.normalize_angle(force_angle - self.theta)
        distance_f = math.sqrt(force[0]**2 + force[1]**2)
        self.move.twist.linear.x = 0.8*min(0.5, distance_f * 0.5) + 0.2*self.move.twist.linear.x
        self.move.twist.angular.z = 0.9*force_angle + 0.1*self.move.twist.angular.z
        self.cmd_pub.publish(self.move)
        self.get_logger().info("Moving along field")

        self.get_logger().info(f"Angle to goal: {delta}     Distance to Goal: {distance}")

        if delta < -tolerance:
            direction.twist.angular.z = -max(min(delta * rotation_gain, -max_rotation_speed), max_rotation_speed)
            direction.twist.linear.x = 0.0
        elif delta > tolerance:
            direction.twist.angular.z = max(min(delta * rotation_gain, -max_rotation_speed), max_rotation_speed)
            direction.twist.linear.x = 0.0
        else:
            direction.twist.linear.x = forward_speed
            direction.twist.angular.z = 0.0

        self.cmd_pub.publish(direction)
        return 0

        '''target_angle = math.atan2(y_final - self.y_odom, x_final - self.x_odom)
        angle_error = target_angle - self.theta

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
        direction.twist.linear.x = linear_vel
        direction.twist.angular.z = angular_vel
        self.cmd_pub.publish(direction)'''

    def stop_robot(self):
        twist = TwistStamped()
        twist.twist.linear.x = 0.0
        twist.twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

'''def main(args=None):
    rclpy.init(args=args)
    node = PotentialField(*rclpy.args)  # Pass arguments (x_goal, y_goal) from the command line
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()'''


def main(args=None):
    rclpy.init(args=args)
    node = PotentialField()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()