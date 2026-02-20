import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_geometry_msgs
import math
from rclpy.duration import Duration
import time
import yaml
import os
from scipy import stats
from std_srvs.srv import Trigger
import cv2

class Mapping(Node):

    def __init__(self):
        super().__init__('mapping_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, 'map', 1)
        self.map_server = self.create_service(Trigger, "map_to_image", self.map_to_image)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.obstacles = []

        self.cell_size = 0.05 # meters
        self.map_x = 16.0
        self.map_y = 16.0
        self.grid = np.full((int(self.map_x / self.cell_size), int(self.map_y / self.cell_size)), -1)

        self.map_timer = self.create_timer(5, self.timer_callback)

    def timer_callback(self):
        self.array_to_map()
        self.grid_to_image()

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        scan = msg.ranges
        num_readings = len(scan)

        max_dist = 3.0 # meters

        for i in range(scan):
            distance = scan[i]
            dist = np.linalg.norm(distance)
            angle = angle_min + self.robot_yaw + angle_increment * i
            if dist <= max_dist:
                ob_x = self.robot_x + dist * math.cos(angle)
                ob_y = self.robot_y + dist * math.sin(angle)
                self.obstacles.append([ob_x, ob_y])

        self.probabilities()
        self.array_to_map()

        self.grid_pub.publish(msg)
        self.get_logger().info('Publishing occupancy grid map')

        #transform = self.tf_buffer.lookup_transform("odom", "laser", rclpy.time.Time())
        #transformed_point = tf2_geometry_msgs.do_transform_pose(self.waypoints[i], transform)
        return 

    def map_to_image(self):
        grid = self.array_to_map()
        self.grid_to_image(grid)

    def world_to_grid(self, x, y):
        map_x = int(x / self.cell_size)
        map_y = int(y / self.cell_size)
        return map_x, map_y

    def bresenham_line(self, start, end):
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1
        steep = abs(dy) > abs(dx)  # determine how steep the line is

        if steep:  # rotate line
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        
        # swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True

        dx = x2 - x1  # recalculate differentials
        dy = y2 - y1  # recalculate differentials
        error = int(dx / 2.0)  # calculate error
        y_step = 1 if y1 < y2 else -1

        # iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = [y, x] if steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += y_step
                error += dx
        
        if swapped:  # reverse the list if the coordinates were swapped
            points.reverse()
        
        points = np.array(points)

        return points
    
    def probabilities(self):

        current_robot_x, current_robot_y = self.world_to_grid(self.robot_x, self.robot_y)

        for i in range(self.obstacles):
            p_occ = stats.norm(loc=self.obstacles[i], scale=1) # probability the sensor saw something
            p_n_occ = 1 - p_occ # probability that the sensor was wrong
            end_x, end_y = self.world_to_grid(self.obstacles[i][0], self.obstacles[i][1])
            beam = self.bresenham_line((current_robot_x, current_robot_y), (end_x, end_y))

            for node in beam:
                self.grid[node[0]][node[1]] = 0.5
                self.grid[node[0]][node[1]] = (p_occ * self.grid[node[0]][node[1]]) / ((p_occ * self.grid[node[0]][node[1]]) + (p_n_occ * (1-self.grid[node[0]][node[1]])))

            self.grid[end_x][end_y] = (p_occ * self.grid[end_x][end_y]) / ((p_occ * self.grid[end_x][end_y]) + (p_n_occ * (1-self.grid[end_x][end_y])))

    def array_to_map(self):
        
        for i in range(self.obstacles[0]):
            for j in range(self.obstacles(1)):
                self.obstacles[i][j] = self.obstacles[i][j] * 100

        int8_array = self.obstacles.astype(np.int8)

        msg = OccupancyGrid()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.cell_size  # meters per cell
        msg.info.width = self.map_y         # number of columns
        msg.info.height = self.map_x        # number of rows

        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = list(int8_array)

        return int8_array

    def grid_to_image(self, occ_grid):
        grid_np = np.array(occ_grid)
        array = grid_np.reshape(self.map_x, self.map_y)
        np.flipud(array)

        scaled_array = array * (255/100)

        image = scaled_array.astype(np.uint8)

        bgr_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        bgr_image[scaled_array == -1] = (135, 206, 235)

        time = self.get_clock().now()

        filename = f'maps/map_to_{time}.PNG'
        script_dir = os.getcwd()
        file_path = os.path.join(script_dir, filename)

        success = cv2.imwrite(file_path, image)

        if success:
            print(f"Image successfully saved")
        else:
            print("Error saving image")


def main(args=None):
    rclpy.init(args=args)
    node = Mapping()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()