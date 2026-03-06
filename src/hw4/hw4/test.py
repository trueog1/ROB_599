import rclpy
from rclpy.node import Node
import numpy as np
import math
import random
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose
from sensor_msgs.msg import LaserScan
import cv2
import os
import pickle
import matplotlib.pyplot as plt
from std_srvs.srv import Empty

class MonteCarloLocalization(Node):
    def __init__(self):
        super().__init__('mcl_node')

        self.num_particles = 500
        self.particles = None
        self.weights = np.ones(self.num_particles) / self.num_particles

        self.map = None
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.last_odom = None
        self.loc_err = []
        self.time_steps = []           
        self.step_counter = 0
        self.max_range = 5.0
        self.current_x = 0.0
        self.current_y = 0.0

        self.motion_noise = {
            'x': 0.1,  # Standard deviation for x motion
            'y': 0.1,  # Standard deviation for y motion
            'theta': 0.05  # Standard deviation for orientation motion
        }

        self.sensor_noise = 0.2  # Standard deviation for sensor model (laser range)

        # Load the map directly from an image file
        filename = f'src/hw4/map.PNG'
        script_dir = os.getcwd()
        map_image_path = os.path.join(script_dir, filename)
        self.load_map_image(map_image_path, resolution=0.1)
        self.picle = f'mapsi.pkl'

        self.load = OccupancyGrid()
        self.load.header.stamp = self.get_clock().now().to_msg()
        self.load.header.frame_id = 'odom'
        self.load.info.resolution = self.map_resolution  # meters per cell
        self.load.info.width = self.map_width         # number of columns
        self.load.info.height = self.map_height       # number of rows
        self.load.info.origin.position.x = self.map_origin[0]
        self.load.info.origin.position.y = self.map_origin[1]
        self.load.info.origin.position.z = 0.0
        self.load.info.origin.orientation.w = 1.0

        # Initialize particles after the map is loaded
        if self.map_data is not None:
            self.initialize_particles()

        # Subscribers 
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/map', 1)

        # Publishers
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 1)
        self.particle_pub = self.create_publisher(PoseArray, '/particle_cloud', 1)

        self.publish_map()

        self.create_service(Empty, 'reset_pose', self.reset_pose_callback)

    def publish_map(self):
        filename = 'mapsi.pkl'
        script_dir = os.getcwd()
        file_path = os.path.join(script_dir, filename)
        with open(file_path, 'rb') as file:
            self.data = pickle.load(file)  
        self.load.data = self.data

        self.grid_pub.publish(self.load)

    def quaternion_from_euler(self, roll, pitch, yaw):
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

    def load_map_image(self, map_image_path, resolution=0.1):
        img = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)

        #print(img)

        if img is None:
            raise ValueError(f"Could not load map image from {map_image_path}")

        self.map_resolution = resolution
        self.map_height, self.map_width = img.shape

        # Convert the image to an occupancy grid (0 for free, 1 for occupied)
        #self.map_data = np.zeros_like(img)
        self.map_data = np.zeros((self.map_height, self.map_width))
        #print(self.map_data)
        self.map_data[img < 20] = 1  # 0 for occupied (black), 255 for free (white) 
        self.map_data[img >= 20] = 0

        filename = f'maps/map_to_.PNG'
        script_dir = os.getcwd()
        file_path = os.path.join(script_dir, filename)

        cv2.imwrite(file_path, img)
        cv2.imwrite('maps/a.PNG', self.map_data * 255)
        #print(self.map_data)

        self.map_data = np.flipud(self.map_data.reshape(self.map_height, self.map_width))  

        self.map_origin = (
            -self.map_width * self.map_resolution / 2,  
            -self.map_height * self.map_resolution / 2  
        )

        self.get_logger().info(f"Map loaded: {self.map_width}x{self.map_height}, resolution={self.map_resolution}m/pixel")

    def initialize_particles(self):
        self.particles = np.zeros((self.num_particles, 3))

        free_cells = np.argwhere(self.map_data == 0)  # 0 = free
        if len(free_cells) == 0:
            self.get_logger().error("No free cells found in the map!")
            return
        
        for i in range(self.num_particles):
            idx = np.random.randint(0, free_cells.shape[0])
            my, mx = free_cells[idx]  
            
            # Convert to world coordinates
            x = mx * self.map_resolution + self.map_origin[0]
            y = my * self.map_resolution + self.map_origin[1]
            
            # Random orientation
            theta = random.uniform(-math.pi, math.pi)
            
            self.particles[i] = [x, y, theta]
        
        self.get_logger().info(f"Initialized {self.num_particles} particles across the map.")

    def world_to_map(self, x, y):
        mx = int((x - self.map_origin[0]) / self.map_resolution)
        my = int(self.map_height - 1 - (y - self.map_origin[1]) / self.map_resolution)  # flip y
        return mx, my

    def map_to_world(self, mx, my):
        x = mx * self.map_resolution + self.map_origin[0]
        y = (self.map_height - 1 - my) * self.map_resolution + self.map_origin[1]  # flip back
        return x, y
    
    '''def world_to_map(self, x, y):
        # Adjust the world coordinates relative to the center of the map origin
        mx = int((x - self.map_origin[0]) / self.map_resolution)
        my = int((y - self.map_origin[1]) / self.map_resolution)
        return mx, my
    
    def map_to_world(self, mx, my):
        """
        Convert map coordinates (mx, my) to world coordinates (x, y).
        mx, my: Map coordinates in pixels or map cells.
        """
        # Map to world conversion
        x = round(float((int(mx) * self.map_resolution) + self.map_origin[0]),2)
        y = round(float((int(my) * self.map_resolution) + self.map_origin[1]),2)

        return x, y'''

    def odom_callback(self, msg):
        if self.last_odom is None:
            self.last_odom = msg
            return
        
        prev_quat = self.last_odom.pose.pose.orientation
        curr_quat = msg.pose.pose.orientation
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        prev_roll, prev_pitch, prev_yaw = self.euler_from_quaternion(prev_quat)
        curr_roll, curr_pitch, curr_yaw = self.euler_from_quaternion(curr_quat)

        delta_theta = curr_yaw - prev_yaw

        delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi

        delta_x = msg.pose.pose.position.x - self.last_odom.pose.pose.position.x
        delta_y = msg.pose.pose.position.y - self.last_odom.pose.pose.position.y

        self.update_particles_with_motion(delta_x, delta_y, delta_theta)

        self.last_odom = msg

        self.get_logger().info(f"Odometry delta_x: {delta_x}, delta_y: {delta_y}, delta_theta: {delta_theta}")

    def update_particles_with_motion(self, delta_x, delta_y, delta_theta):
        for i in range(self.num_particles):
            x, y, theta = self.particles[i]

            delta_x_noise = random.gauss(delta_x, self.motion_noise['x'])
            delta_y_noise = random.gauss(delta_y, self.motion_noise['y'])
            delta_theta_noise = random.gauss(delta_theta, self.motion_noise['theta'])

            new_x = x + delta_x_noise
            new_y = y + delta_y_noise
            new_theta = (theta + delta_theta_noise) % (2 * math.pi)

            self.particles[i] = [new_x, new_y, new_theta]

    def scan_callback(self, msg):
        if self.particles is None or self.map_data is None:
            return

        self.get_logger().info("Scan callback triggered.")

        log_weights = np.zeros(self.num_particles)

        for i in range(self.num_particles):
            x, y, theta = self.particles[i]
            #print(theta)
            log_w = 0.0

            for j in range(0, len(msg.ranges), 2):  
                z = msg.ranges[j]

                if math.isinf(z) or math.isnan(z):
                    continue

                beam_angle = theta + msg.angle_min + j * msg.angle_increment
                z_expected = self.ray_cast(x, y, beam_angle)
                #self.get_logger().info(f"Z: {z}    Z expected: {z_expected}")

                p_hit = self.gaussian_sensor_model(z, z_expected)
                p_hit = max(p_hit, 1e-6)

                #print(p_hit)

                log_w += math.log(p_hit)

            log_weights[i] = log_w

        #self.get_logger().info(f"Log Weights: {log_weights}")

        log_max = np.max(log_weights)
        weights = np.exp(log_weights - log_max)
        weights /= np.sum(weights)
        self.weights = weights
        #self.get_logger().info(f"Normal Weights: {self.weights}")

        self.resample()

        self.publish_most_likely_pose()

        self.publish_particles()

        self.compute_localization_error(self.current_x, self.current_y)

    def gaussian_sensor_model(self, z, z_expected):
        return math.exp(-0.5 * ((z - z_expected) / self.sensor_noise) ** 2)

    def ray_cast(self, x, y, theta):
        mx0, my0 = self.world_to_map(x, y)

        max_cells = int(self.max_range / self.map_resolution)

        mx1 = int(mx0 + max_cells * math.cos(theta))
        my1 = int(my0 + max_cells * math.sin(theta))

        dx = abs(mx1 - mx0)
        dy = abs(my1 - my0)
        sx = 1 if mx0 < mx1 else -1
        sy = 1 if my0 < my1 else -1
        err = dx - dy

        mx, my = mx0, my0

        for _ in range(max_cells):
            if mx < 0 or my < 0 or mx >= self.map_width or my >= self.map_height:
                break

            if self.map_data[my, mx] == 1:  # occupied
                wx, wy = self.map_to_world(mx, my)
                return math.sqrt((wx - x) ** 2 + (wy - y) ** 2)

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                mx += sx
            if e2 < dx:
                err += dx
                my += sy

        wx, wy = self.map_to_world(mx, my)
        return min(math.sqrt((wx - x) ** 2 + (wy - y) ** 2), self.max_range)

    def publish_particles(self):
        if self.particles is not None:
            particle_array = PoseArray()
            particle_array.header.frame_id = "odom"

            for p in self.particles:
                pose = Pose()
                pose.position.x = p[0]
                pose.position.y = p[1]
                pose.position.z = 0.01
                q = self.quaternion_from_euler(0, 0, p[2])
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                particle_array.poses.append(pose)

            self.particle_pub.publish(particle_array)
            #print(particle_array)
            self.get_logger().info("Particle cloud published.")
            self.get_logger().info(f"Published {len(particle_array.poses)} particles.")

    def resample(self):
        normalized_weights = self.weights / np.sum(self.weights)

        cumulative_sum = np.cumsum(normalized_weights)

        N = len(self.particles)

        random_start = np.random.uniform(0, 1 / N)

        indices = []
        i = 0
        for j in range(N):
            threshold = random_start + j / N
            while cumulative_sum[i] < threshold:
                i += 1
            indices.append(i)

        self.particles = self.particles[indices]
        self.weights = np.ones(N) / N  
        self.get_logger().info("Resampling occured.")

    def get_most_likely_pose(self):
        most_likely_index = np.argmax(self.weights)
        
        most_likely_pose = self.particles[most_likely_index]
        
        x, y, theta = most_likely_pose

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "odom"

        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y

        q = self.quaternion_from_euler(0, 0, theta)
        pose_msg.pose.pose.orientation.x = q[0]
        pose_msg.pose.pose.orientation.y = q[1]
        pose_msg.pose.pose.orientation.z = q[2]
        pose_msg.pose.pose.orientation.w = q[3]

        return pose_msg
    
    def publish_most_likely_pose(self):
        most_likely_pose = self.get_most_likely_pose()
        self.pose_pub.publish(most_likely_pose)
        self.get_logger().info("Most likely pose published.")
        return most_likely_pose.pose.pose.position.x, most_likely_pose.pose.pose.position.y
    
    def compute_localization_error(self, true_x, true_y):
        pose_msg = self.get_most_likely_pose()
        est_x = pose_msg.pose.pose.position.x
        est_y = pose_msg.pose.pose.position.y

        error = math.sqrt((est_x - true_x)**2 + (est_y - true_y)**2)
        
        self.loc_err.append(error)
        self.time_steps.append(self.step_counter)
        self.step_counter += 1

    def plot_localization_error(self):
        plt.figure()
        plt.plot(self.time_steps, self.loc_err, '-o')
        plt.xlabel('Time step')
        plt.ylabel('Localization error (m)')
        plt.title('Monte Carlo Localization Error Over Time')
        plt.grid(True)
        plt.show(block=True)

    def reset_pose_callback(self, request, response):
        self.get_logger().info("Resetting pose estimate and reinitializing particles...")

        self.initialize_particles()

        self.weights = np.ones(self.num_particles) / self.num_particles

        self.loc_err = []

        return response


def main(args=None):
    rclpy.init(args=args)
    node = MonteCarloLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Ctrl+C detected. Plotting localization error...")
        node.plot_localization_error()  
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()