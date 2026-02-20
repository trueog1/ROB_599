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
import pickle

class Mapping(Node):

    def __init__(self):
        super().__init__('mapping_node')

        #self.tf_buffer = tf2_ros.Buffer()
        #self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, 'base_scan', self.scan_callback, 10)
        #self.subscription = self.create_subscription(OccupancyGrid,'/map',self.map_callback,10)
        self.grid_pub = self.create_publisher(OccupancyGrid, 'map', 1)
        self.map_server = self.create_service(Trigger, "map_to_image", self.map_to_image)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.obstacles = []

        self.cell_size = 0.1 # meters
        self.map_x = 16.0
        self.map_y = 16.0
        self.grid_x = int(self.map_x / self.cell_size)
        self.grid_y = int(self.map_y / self.cell_size)
        self.grid = np.full((self.grid_x, self.grid_y), -1.0)

        self.msg = OccupancyGrid()
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.header.frame_id = 'odom'
        self.msg.info.resolution = self.cell_size  # meters per cell
        self.msg.info.width = self.grid_x         # number of columns
        self.msg.info.height = self.grid_y        # number of rows
        self.msg.info.origin.position.x = -8.0
        self.msg.info.origin.position.y = -8.0
        self.msg.info.origin.position.z = 0.0
        self.msg.info.origin.orientation.w = 1.0

        self.map_timer = self.create_timer(5, self.timer_callback)

    def timer_callback(self):
        self.array_to_map()
        self.grid_to_image()
        with open('map.pkl', 'wb') as f:
            pickle.dump(self.msg.data, f)
        self.get_logger().info('Map saved to map.pkl')

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
        angle_max = msg.angle_max
        scan = np.array(msg.ranges)
        max_dist = 4.5 # meters

        num_readings = len(scan)
        theta = angle_min + self.robot_yaw + angle_increment * np.arange(num_readings)
        valid_mask = (scan <= max_dist) & (scan > 0) # Added >0 to filter out invalid/zero range
        dist_valid = scan[valid_mask]
        angles_valid = theta[valid_mask]
        ob_x = self.robot_x + dist_valid * np.cos(angles_valid)
        ob_y = self.robot_y + dist_valid * np.sin(angles_valid)
        self.obstacles = np.column_stack((ob_x, ob_y))
        self.get_logger().info('Got distance')

        self.probabilities()
        #self.get_logger().info('Prob Good')
        self.array_to_map()
        #self.get_logger().info('Array to Map good')
        self.grid_pub.publish(self.msg)

        self.get_logger().info('Publishing occupancy grid map')

        #transform = self.tf_buffer.lookup_transform("odom", "laser", rclpy.time.Time())
        #transformed_point = tf2_geometry_msgs.do_transform_pose(self.waypoints[i], transform)
        return 

    def map_to_image(self, request, response):
        self.array_to_map()
        self.grid_to_image()
        response.success = True
        response.message = 'Image created'
        return response

    def world_to_grid(self, x, y):
        map_x = int((x + self.msg.info.origin.position.x) / self.cell_size)
        map_y = int((y + self.msg.info.origin.position.y) / self.cell_size)
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
        xx = []
        yy = []
        for x in range(x1, x2 + 1):
            coord = [y, x] if steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += y_step
                error += dx
        
        if swapped:  # reverse the list if the coordinates were swapped
            points.reverse()
        
        for i in range(len(points)):
            xx.append(points[i][0])
            yy.append(points[i][1])

        xx_f = np.array(xx)
        yy_f = np.array(yy)

        return xx_f, yy_f
    
    def probabilities(self):
        current_robot_x, current_robot_y = self.world_to_grid(self.robot_x, self.robot_y)
        #print(current_robot_x, current_robot_y)

        #current_robot_x, current_robot_y = self.world_to_grid(self.robot_x, self.robot_y)
        #print(self.obstacles)
        for i in range(len(self.obstacles)):
            end_x, end_y = self.world_to_grid(self.obstacles[i][0], self.obstacles[i][1])

            x, y = self.bresenham_line((current_robot_x, current_robot_y), (end_x, end_y))

            d = np.sqrt((self.obstacles[i][0] - self.robot_x)**2 + (self.obstacles[i][1] - self.robot_y)**2)

            sensor_dist = np.sqrt((x - current_robot_x)**2 + (y - current_robot_y)**2) * self.cell_size

            valid_indices = (x >= -self.grid_x) & (x < self.grid_x) & (y >= -self.grid_y) & (y < self.grid_y)

            valid_x = x[valid_indices]
            valid_y = y[valid_indices]
            valid_sensor_dist = sensor_dist[valid_indices]

            temp_values = self.grid[valid_y, valid_x]
            temp_values = np.where(temp_values > 0, temp_values, 0.5)

            p_occ = np.exp(-0.5 * ((valid_sensor_dist - d) / 0.1) ** 2)

            prob_values = (p_occ * temp_values) / ((p_occ * temp_values) + ((1 - p_occ) * (1 - temp_values)))
            prob_values = np.clip(prob_values, 0.001, 0.999)

            self.grid[valid_y, valid_x] = prob_values
           
        self.get_logger().info('Prob Good')

    def array_to_map(self):
        occ_map = -np.ones((self.grid_x, self.grid_y), dtype=np.int8)
        mask_of_known = self.grid >= 0
        scale_range = np.clip(self.grid[mask_of_known], 0, 1) * 100
        occ_map[mask_of_known] = scale_range.astype(np.int8)

        self.msg.data = occ_map.flatten().tolist()

    def grid_to_image(self):
        grid_data = np.array(self.msg.data)  # Extract data from OccupancyGrid message
        array = grid_data.reshape(self.grid_y, self.grid_x)  # Reshape to 2D array (height x width)

        # Flip the array upside down if necessary (for correct orientation)
        array = np.flipud(array)

        # Create an empty RGB image (3 channels for color)
        rgb_image = np.zeros((array.shape[0], array.shape[1], 3), dtype=np.uint8)

        # Handle free and occupied cells with smooth grayscale
        mask_free_or_occupied = (array >= 0) & (array != -1)  # Exclude unknowns
        scaled_values = ((100 - array[mask_free_or_occupied]) / 100.0 * 255).astype(np.uint8)
        rgb_image[mask_free_or_occupied] = np.dstack([scaled_values, scaled_values, scaled_values])  # Grayscale

        # Handle unknowns (value -1) → blue color
        rgb_image[array == -1] = (0, 0, 255)  # Blue for unknowns

        # Debugging: Check unique values in the array
        print("Unique values in array:", np.unique(array))
        print("Min value in array:", array.min())
        print("Max value in array:", array.max())

        # Generate a filename with the current time
        time = self.get_clock().now().seconds_nanoseconds
        filename = f'maps/map_to_{time}.PNG'
        script_dir = os.getcwd()
        file_path = os.path.join(script_dir, filename)

        # Save the image using OpenCV, converting from RGB to BGR for OpenCV compatibility
        success = cv2.imwrite(file_path, cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))

        # Check if the image was saved successfully
        if success:
            print(f"Image successfully saved at {file_path}")
        else:
            print("Error saving image")


def main(args=None):
    rclpy.init(args=args)
    node = Mapping()
    rclpy.spin(node)
    node.destroy_node() #
    rclpy.shutdown()

if __name__ == '__main__':
    main()