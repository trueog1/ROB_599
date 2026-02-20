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

def world_to_grid(x, y):
        map_x = int((x + (-8.0)) / 0.1)
        map_y = -int((y + (-8.0)) / 0.1)
        return map_x, map_y

def create_node(position, g= float('inf'), h= 0.0, parent = None):
        """
        Create a node for the A* algorithm.
    
        Args:
            position: (x, y) coordinates of the node
            g: Cost from start to this node (default: infinity)
            h: Estimated cost from this node to goal (default: 0)
            parent: Parent node (default: None)
    
        Returns:
            Dictionary containing node information
        """
        return {'position': position,'g': g,'h': h,'f': g + h,'parent': parent}
    
'''def calculate_heuristic(pose1, pose2):
        """
        Calculate the estimated distance between two points using Euclidean distance.
        """
        x1, y1 = pose1
        x2, y2 = pose2
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
def get_valid_neighbors(grid, position):
        """
        Get all valid neighboring positions in the grid.
    
        Args:
            grid: 2D numpy array where 0 represents walkable cells and 1 represents obstacles
            position: Current position (x, y)
    
        Returns:
            List of valid neighboring positions
        """
        x, y = position
        rows, cols = grid.shape
    
        # All possible moves (including diagonals)
        possible_moves = [
            (x+1, y), (x-1, y),    # Right, Left
            (x, y+1), (x, y-1),    # Up, Down
            (x+1, y+1), (x-1, y-1),  # Diagonal moves
            (x+1, y-1), (x-1, y+1)
        ]
    
        return [
            (nx, ny) for nx, ny in possible_moves
            if 0 <= nx < rows and 0 <= ny < cols  # Within grid bounds
            and grid[nx, ny] == 0                # Not an obstacle
        ]
    
def reconstruct_path(goal_node):
        """
        Reconstruct the path from goal to start by following parent pointers.
        """
        path = []
        current = goal_node
    
        while current is not None:
            path.append(current['position'])
            current = current['parent']
        
        return path[::-1]  # Reverse to get path from start to goal
    
def find_path(grid, start, goal):
        """
        Find the optimal path using A* algorithm.
    
        Args:
            grid: 2D numpy array (0 = free space, 1 = obstacle)
            start: Starting position (x, y)
            goal: Goal position (x, y)
    
        Returns:
            List of positions representing the optimal path
        """
        # Initialize start node
        start_node = create_node(position=start,g=0,h=calculate_heuristic(start, goal))
    
        # Initialize open and closed sets
        open_list = [(start_node['f'], start)]  # Priority queue
        open_dict = {start: start_node}         # For quick node lookup
        closed_set = set()                      # Explored nodes
    
        while open_list:
            # Get node with lowest f value
            _, current_pos = heapq.heappop(open_list)
            current_node = open_dict[current_pos]
        
            # Check if we've reached the goal
            if current_pos == goal:
                return reconstruct_path(current_node)
            
            closed_set.add(current_pos)
        
            # Explore neighbors
            for neighbor_pos in get_valid_neighbors(grid, current_pos):
                # Skip if already explored
                if neighbor_pos in closed_set:
                    continue
                
                # Calculate new path cost
                tentative_g = current_node['g'] + calculate_heuristic(current_pos, neighbor_pos)
            
                # Create or update neighbor
                if neighbor_pos not in open_dict:
                    neighbor = create_node(position=neighbor_pos,g=tentative_g,h=calculate_heuristic(neighbor_pos, goal),parent=current_node)
                    heapq.heappush(open_list, (neighbor['f'], neighbor_pos))
                    open_dict[neighbor_pos] = neighbor
                
                elif tentative_g < open_dict[neighbor_pos]['g']:
                    # Found a better path to the neighbor
                    neighbor = open_dict[neighbor_pos]
                    neighbor['g'] = tentative_g
                    neighbor['f'] = tentative_g + neighbor['h']
                    neighbor['parent'] = current_node
    
        return []  # No path found'''

'''class Cell:
    def __init__(self):
      # Parent cell's row index
        self.parent_i = 0
    # Parent cell's column index
        self.parent_j = 0
 # Total cost of the cell (g + h)
        self.f = float('inf')
    # Cost from start to this cell
        self.g = float('inf')
    # Heuristic cost from this cell to destination
        self.h = 0


# Define the size of the grid
ROW = 160
COL = 160

# Check if a cell is valid (within the grid)


def is_valid(row, col):
    return (row >= -ROW) and (row < ROW) and (col >= -COL) and (col < COL)

# Check if a cell is unblocked


def is_unblocked(grid, row, col):
    return grid[row][col] == 1

# Check if a cell is the destination


def is_destination(row, col, dest):
    return row == dest[0] and col == dest[1]

# Calculate the heuristic value of a cell (Euclidean distance to destination)


def calculate_h_value(row, col, dest):
    return ((row - dest[0]) ** 2 + (col - dest[1]) ** 2) ** 0.5

# Trace the path from source to destination


def trace_path(cell_details, dest):
    print("The Path is ")
    path = []
    row = dest[0]
    col = dest[1]

    # Trace the path from destination to source using parent cells
    while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
        path.append((row, col))
        temp_row = cell_details[row][col].parent_i
        temp_col = cell_details[row][col].parent_j
        row = temp_row
        col = temp_col

    # Add the source cell to the path
    path.append((row, col))
    # Reverse the path to get the path from source to destination
    path.reverse()

    # Print the path
    for i in path:
        print("->", i, end=" ")
    print()

# Implement the A* search algorithm

def a_star_search(grid, src, dest):
    # Check if the source and destination are valid
    if not is_valid(src[0], src[1]) or not is_valid(dest[0], dest[1]):
        print("Source or destination is invalid")
        return

    # Check if the source and destination are unblocked
    if not is_unblocked(grid, src[0], src[1]) or not is_unblocked(grid, dest[0], dest[1]):
        print("Source or the destination is blocked")
        return

    # Check if we are already at the destination
    if is_destination(src[0], src[1], dest):
        print("We are already at the destination")
        return

    # Initialize the closed list (visited cells)
    closed_list = [[False for _ in range(COL)] for _ in range(ROW)]
    # Initialize the details of each cell
    cell_details = [[Cell() for _ in range(COL)] for _ in range(ROW)]

    # Initialize the start cell details
    i = src[0]
    j = src[1]
    cell_details[i][j].f = 0
    cell_details[i][j].g = 0
    cell_details[i][j].h = 0
    cell_details[i][j].parent_i = i
    cell_details[i][j].parent_j = j

    # Initialize the open list (cells to be visited) with the start cell
    open_list = []
    heapq.heappush(open_list, (0.0, i, j))

    # Initialize the flag for whether destination is found
    found_dest = False

    # Main loop of A* search algorithm
    while len(open_list) > 0:
        # Pop the cell with the smallest f value from the open list
        p = heapq.heappop(open_list)

        # Mark the cell as visited
        i = p[1]
        j = p[2]
        closed_list[i][j] = True

        # For each direction, check the successors
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0),
                      (1, 1), (1, -1), (-1, 1), (-1, -1)]
        for dir in directions:
            new_i = i + dir[0]
            new_j = j + dir[1]

            # If the successor is valid, unblocked, and not visited
            if is_valid(new_i, new_j) and is_unblocked(grid, new_i, new_j) and not closed_list[new_i][new_j]:
                # If the successor is the destination
                if is_destination(new_i, new_j, dest):
                    # Set the parent of the destination cell
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j
                    print("The destination cell is found")
                    # Trace and print the path from source to destination
                    trace_path(cell_details, dest)
                    found_dest = True
                    return
                else:
                    # Calculate the new f, g, and h values
                    g_new = cell_details[i][j].g + 1.0
                    h_new = calculate_h_value(new_i, new_j, dest)
                    f_new = g_new + h_new

                    # If the cell is not in the open list or the new f value is smaller
                    if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                        # Add the cell to the open list
                        heapq.heappush(open_list, (f_new, new_i, new_j))
                        # Update the cell details
                        cell_details[new_i][new_j].f = f_new
                        cell_details[new_i][new_j].g = g_new
                        cell_details[new_i][new_j].h = h_new
                        cell_details[new_i][new_j].parent_i = i
                        cell_details[new_i][new_j].parent_j = j

    # If the destination is not found after visiting all cells
    if not found_dest:
        print("Failed to find the destination cell")'''
def grid_to_world(x, y):
    new_x = round(float((int(x) * 0.1) + (-8.0)), 2)
    new_y = round(float((-int(y) * 0.1) + (-8.0)), 2)

    return new_x, new_y

ROW = 160
COL = 160

def is_valid(row, col):
    return (row >= -ROW) and (row < ROW) and (col >= -COL) and (col < COL)


def is_unblocked(grid, row, col):
    return grid[row][col] == 0


def is_destination(row, col, dest):
    return (row, col) == dest


def calculate_h_value(row, col, dest):
    # Euclidean distance (correct for diagonal movement)
    return math.sqrt((row - dest[0]) ** 2 + (col - dest[1]) ** 2)


def trace_path(parent, dest):
    print("The Path is")
    path = []
    current = dest

    while current in parent:
        path.append(current)
        current = parent[current]

    path.append(current)  # Add source
    path.reverse()

    for p in path:
        print("->", p, end=" ")
    print()

    return path


def a_star_search(grid, src, dest):

    if not is_valid(*src) or not is_valid(*dest):
        print("Source or destination is invalid")
        return

    if not is_unblocked(grid, *src) or not is_unblocked(grid, *dest):
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

            if not is_valid(new_row, new_col):
                continue
            if not is_unblocked(grid, new_row, new_col):
                continue
            if neighbor in closed_set:
                continue

            # Determine movement cost
            if dr != 0 and dc != 0:
                move_cost = math.sqrt(2)  # Diagonal
            else:
                move_cost = 1.0  # Straight

            g_new = g_cost[current] + move_cost

            if is_destination(new_row, new_col, dest):
                parent[neighbor] = current
                print("The destination cell is found")
                return trace_path(parent, dest)

            if neighbor not in g_cost or g_new < g_cost[neighbor]:
                g_cost[neighbor] = g_new
                h_new = calculate_h_value(new_row, new_col, dest)
                f_new = g_new + h_new
                heapq.heappush(open_list, (f_new, neighbor))
                parent[neighbor] = current

    print("Failed to find the destination cell")
    return None
# Driver Code

filename = 'ros2_ws_ROB599/src/hw3/map.pkl'
script_dir = os.getcwd()
file_path = os.path.join(script_dir, filename)
with open(file_path, 'rb') as file:
    msg = pickle.load(file)

grid_data = np.array(msg)  # Extract data from OccupancyGrid message
#print(grid_data)
array = grid_data.reshape(160, 160)  # Reshape to 2D array (height x width)
#inflate_grid = np.full((160, 160), -1.0)

# Flip the array upside down if necessary (for correct orientation)
array = np.flipud(array)

boolean_arr = (array > 90)
radius = 2
kernel_size = 2 * radius + 1
# Elliptical shape creates a circle when width == height
circular_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

new_map = scipy.ndimage.binary_dilation(boolean_arr, structure=circular_kernel).astype(int)
#print(new_map)
#inflate_grid[:] = new_map
#print(inflate_grid)
mask = (array == -1) & (new_map == 0)
new_map[mask] = -1

rgb_image = np.zeros((new_map.shape[0], new_map.shape[1], 3), dtype=np.uint8)

        # Handle free and occupied cells with smooth grayscale
mask_free_or_occupied = (new_map == 0) & (array != -1) # Exclude unknowns
scaled_values = ((100 - new_map[mask_free_or_occupied]) / 100.0 * 255).astype(np.uint8)
rgb_image[mask_free_or_occupied] = np.dstack([scaled_values, scaled_values, scaled_values]) 
rgb_image[new_map == -1] = (0, 0, 255)

filename = f'ros2_ws_ROB599/src/hw3/maps_test.PNG'
script_dir = os.getcwd()
file_path = os.path.join(script_dir, filename)

        # Save the image using OpenCV, converting from RGB to BGR for OpenCV compatibility
success = cv2.imwrite(file_path, cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))

#print("Circular Kernel (Radius 3):\n", circular_kernel)
#print(boolean_arr)

s_x, s_y = world_to_grid(0, 0)
g_x, g_y = world_to_grid(3, -1)
print(new_map[s_x][s_y])
print(new_map[g_y][g_x])
src = (s_y, s_x)
des = (g_y, g_x)
# Find the path
path = a_star_search(new_map, src, des)
print(path)
world_path = []
waypoints = []
for i in range(len(path)):
    x, y = grid_to_world(path[i][0], path[i][1])
    world_path.append((x, y))

print(world_path)
waypoints = world_path.copy()

'''for i in range(len(world_path)):
    new = tuple(float(x) for x in world_path[i])
    waypoints.append(new)'''

print(waypoints)

for i in range(len(path)):
    new_map[path[i][0]][path[i][1]] = -2

mask = (array == -1) & (new_map == 0)
new_map[mask] = -1

rgb_image = np.zeros((new_map.shape[0], new_map.shape[1], 3), dtype=np.uint8)

        # Handle free and occupied cells with smooth grayscale
mask_free_or_occupied = (new_map == 0) & (array != -1) # Exclude unknowns
scaled_values = ((100 - new_map[mask_free_or_occupied]) / 100.0 * 255).astype(np.uint8)
rgb_image[mask_free_or_occupied] = np.dstack([scaled_values, scaled_values, scaled_values]) 
rgb_image[new_map == -1] = (0, 0, 255)
rgb_image[new_map == -2] = (254,32,32)

filename = f'ros2_ws_ROB599/src/hw3/maps_test_path.PNG'
script_dir = os.getcwd()
file_path = os.path.join(script_dir, filename)

        # Save the image using OpenCV, converting from RGB to BGR for OpenCV compatibility
success = cv2.imwrite(file_path, cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))

sp = waypoints[::5]
waypoints_data = {'waypoints':[]}
for i in range(len(sp)):
    position = {'x': -sp[i][1], 'y': -sp[i][0], 'z': 0.0}
    pose = {'position': position}
    dict = {"frame_id": "odom", 'pose': pose}
    waypoints_data['waypoints'].append(dict)

yaml_file_path = 'waypoint.yaml'
with open(yaml_file_path, 'w') as file:
    # Use yaml.dump() to convert the Python dictionary to YAML format
    # default_flow_style=False ensures the data is written in a readable block style
    yaml.dump(waypoints_data, file, sort_keys=False, indent=2)

print(f"Successfully wrote waypoints to {yaml_file_path}")

'''x1, y1 = (0,0)
x2, y2 = (-5, 6)
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
        
    points = np.array(points)'''