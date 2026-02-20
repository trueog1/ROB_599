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
import scipy
from std_srvs.srv import Trigger
import cv2
import pickle
import heapq

class PathPlan(Node):

    def __init__(self):
        super().__init__('path_planning_node')

        self.x_odom = 0.0
        self.y_odom = 0.0
        self.theta = 0.0

        self.cell_size = 0.1 # meters
        self.map_x = 16.0
        self.map_y = 16.0
        self.grid_x = int(self.map_x / self.cell_size)
        self.grid_y = int(self.map_y / self.cell_size)
        self.grid = np.full((self.grid_x, self.grid_y), -1.0)
        self.inflate_grid = np.zeros((self.grid_x, self.grid_y))
        self.array = np.zeros((self.grid_x, self.grid_y))
        self.start = (0, 0)
        self.goal = (3, 6)
        self.waypoints = []

        self.msg = OccupancyGrid()
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.header.frame_id = 'odom'
        self.msg.info.resolution = self.cell_size  # meters per cell
        self.msg.info.width = self.grid_x         # number of columns
        self.msg.info.height = self.grid_y        # number of rows
        self.msg.info.origin.position.x = -8.0
        self.msg.info.origin.position.y = -8.0
        
        filename = 'map.pkl'
        script_dir = os.getcwd()
        file_path = os.path.join(script_dir, filename)
        with open(file_path, 'rb') as file:
            self.map = pickle.load(file)  

        self.map_to_bolarray()
        path = self.path_to_map()
        self.path_image(path)
        self.create_yaml()


    '''To get the robot to follow your path, you'll need to do the following steps: 
    (1) ingest a map; x
    (2) threshold to boolean occupancies; x
    (3) inflate each cell so that you have a configuration space; x
    (4) plan the path; 
    (5) feed waypoints to your robot to follow the path.'''

    def world_to_grid(self, x, y):
        map_x = int((x + self.msg.info.origin.position.x) / self.cell_size)
        map_y = -int((y + self.msg.info.origin.position.y) / self.cell_size)
        return map_x, map_y
    
    def grid_to_world(self, x, y):
        new_x = round(float((int(x) * self.cell_size) + self.msg.info.origin.position.x),2)
        new_y = round(float((-int(y) * self.cell_size) + self.msg.info.origin.position.y),2)
        return new_x, new_y

    def map_to_bolarray(self):
        grid_data = np.array(self.map)  # Extract data from OccupancyGrid message
        array = grid_data.reshape(self.grid_y, self.grid_x)  # Reshape to 2D array (height x width)
        array = np.flipud(array)
        self.array[:] = array
        boolean_arr = (array > 90)
        
        radius = 2
        kernel_size = 2 * radius + 1
        # Elliptical shape creates a circle when width == height
        circular_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        new_map = scipy.ndimage.binary_dilation(boolean_arr, structure=circular_kernel).astype(int)
        #print(new_map)
        self.inflate_grid[:] = new_map

        mask = (array == -1) & (new_map == 0)
        new_map[mask] = -1
        rgb_image = np.zeros((new_map.shape[0], new_map.shape[1], 3), dtype=np.uint8)

        # Handle free and occupied cells with smooth grayscale
        mask_free_or_occupied = (new_map == 0) & (array != -1) # Exclude unknowns
        scaled_values = ((100 - new_map[mask_free_or_occupied]) / 100.0 * 255).astype(np.uint8)
        rgb_image[mask_free_or_occupied] = np.dstack([scaled_values, scaled_values, scaled_values]) 
        rgb_image[array == -1] = (0, 0, 255)

        filename = f'map_inflate.PNG'
        script_dir = os.getcwd()
        file_path = os.path.join(script_dir, filename)

        # Save the image using OpenCV, converting from RGB to BGR for OpenCV compatibility
        success = cv2.imwrite(file_path, cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))
        if success:
            print(f"Image successfully saved at {file_path}")
        else:
            print("Error saving image")

    def is_valid(self, row, col):
        return (row >= -self.grid_x) and (row < self.grid_x) and (col >= -self.grid_y) and (col < self.grid_y)

    def is_unblocked(self, grid, row, col):
        return grid[row][col] == 0

    def is_destination(self, row, col, dest):
        return (row, col) == dest

    def calculate_h_value(self, row, col, dest):
        # Euclidean distance (correct for diagonal movement)
        return math.sqrt((row - dest[0]) ** 2 + (col - dest[1]) ** 2)

    def trace_path(self, parent, dest):
        print("The Path is")
        path = []
        current = dest

        while current in parent:
            path.append(current)
            current = parent[current]

        path.append(current)  # Add source
        path.reverse()

        '''for p in path:
            print("->", p, end=" ")
        print()'''

        return path

    def a_star_search(self, grid, src, dest):

        if not self.is_valid(*src) or not self.is_valid(*dest):
            print("Source or destination is invalid")
            return

        if not self.is_unblocked(grid, *src) or not self.is_unblocked(grid, *dest):
            print("Source or destination is blocked")
            return

        if src == dest:
            print("We are already at the destination")
            return

        open_list = []
        heapq.heappush(open_list, (0, src))

        closed_set = set()
        g_cost = {src: 0}
        parent = {}

        directions = [
            (0, 1), (0, -1), (1, 0), (-1, 0),      # Straight
            (1, 1), (1, -1), (-1, 1), (-1, -1)    # Diagonal
        ]

        while open_list:
            current_f, current = heapq.heappop(open_list)

            if current in closed_set:
                continue

            closed_set.add(current)
            row, col = current

            for dr, dc in directions:
                new_row = row + dr
                new_col = col + dc
                neighbor = (new_row, new_col)

                if not self.is_valid(new_row, new_col):
                    continue
                if not self.is_unblocked(grid, new_row, new_col):
                    continue
                if neighbor in closed_set:
                    continue

                # Determine movement cost
                if dr != 0 and dc != 0:
                    move_cost = math.sqrt(2)  # Diagonal
                else:
                    move_cost = 1.0  # Straight

                g_new = g_cost[current] + move_cost

                if self.is_destination(new_row, new_col, dest):
                    parent[neighbor] = current
                    print("The destination cell is found")
                    return self.trace_path(parent, dest)

                if neighbor not in g_cost or g_new < g_cost[neighbor]:
                    g_cost[neighbor] = g_new
                    h_new = self.calculate_h_value(new_row, new_col, dest)
                    f_new = g_new + h_new
                    heapq.heappush(open_list, (f_new, neighbor))
                    parent[neighbor] = current

        print("Failed to find the destination cell")
        return None
    
    def path_to_map(self):
        s_x, s_y = self.world_to_grid(self.start[0], self.start[1])
        g_x, g_y = self.world_to_grid(self.goal[0], self.goal[1])
        #print(self.inflate_grid[s_x][s_y])
        #print(self.inflate_grid[g_y][g_x])
        src = (s_y, s_x)
        des = (g_y, g_x)
        # Find the path
        path = self.a_star_search(self.inflate_grid, src, des)

        world_path = []
        for i in range(len(path)):
            x, y = self.grid_to_world(path[i][0], path[i][1])
            world_path.append((x, y))
        self.waypoints = world_path.copy()
        
        return path

    def path_image(self, path):
        for i in range(len(path)):
            self.inflate_grid[path[i][0]][path[i][1]] = -2

        mask = (self.array == -1) & (self.inflate_grid == 0)
        self.inflate_grid[mask] = -1

        rgb_image = np.zeros((self.inflate_grid.shape[0], self.inflate_grid.shape[1], 3), dtype=np.uint8)

        # Handle free and occupied cells with smooth grayscale
        mask_free_or_occupied = (self.inflate_grid == 0) & (self.array != -1) # Exclude unknowns
        scaled_values = ((100 - self.inflate_grid[mask_free_or_occupied]) / 100.0 * 255).astype(np.uint8)
        rgb_image[mask_free_or_occupied] = np.dstack([scaled_values, scaled_values, scaled_values]) 
        rgb_image[self.inflate_grid == -1] = (0, 0, 255)
        rgb_image[self.inflate_grid == -2] = (254,32,32)

        filename = f'maps_path.PNG'
        script_dir = os.getcwd()
        file_path = os.path.join(script_dir, filename)

        # Save the image using OpenCV, converting from RGB to BGR for OpenCV compatibility
        success = cv2.imwrite(file_path, cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))

    def create_yaml(self):
        saved_points = self.waypoints[::5]
        waypoints_data = {'waypoints':[]}
        for i in range(len(saved_points)):
            position = {'x': -saved_points[i][1], 'y': -saved_points[i][0], 'z': 0.0}
            pose = {'position': position}
            dict = {'frame_id': 'odom', 'pose': pose}
            waypoints_data['waypoints'].append(dict)

        yaml_file_path = 'hw3/waypoint.yaml'
        with open(yaml_file_path, 'w') as file:
            # Use yaml.dump() to convert the Python dictionary to YAML format
            # default_flow_style=False ensures the data is written in a readable block style
            yaml.dump(waypoints_data, file, default_flow_style=False, sort_keys=False)

        print(f"Successfully wrote waypoints to {yaml_file_path}")
        

    
def main(args=None):
    rclpy.init(args=args)
    node = PathPlan()
    rclpy.spin(node)
    node.destroy_node() #
    rclpy.shutdown()

if __name__ == '__main__':
    main()