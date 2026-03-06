import rclpy
import yaml
import os
import numpy as np

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from geometry_msgs.msg import Quaternion, Pose, Point, PoseStamped, PoseArray
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from ament_index_python.packages import get_package_share_directory
from math import degrees
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from typing import List
import cv2
import math
import scipy

class Particle:
    def __init__(self, pose: Pose, weight: float):
        self.pose: Pose = pose
        self.weight: float = weight

class Map:
    def __init__(self, logger):
        package_dir = get_package_share_directory('hw4')
        self.image_name = 'maps_inflate.PNG'
        self.resolution = 0.1
        self.origin = [0.0, 0.0, 0.0]
        self.negate = 0
        self.occupied_thresh = 0.65
        self.free_thresh = 0.25
        image_path = os.path.join(package_dir + self.image_name)
        image_data = cv2.imread(image_path, -1)
        rotated = np.rot90(image_data)
        self.data = []
        for pixel in list(rotated.flatten()):
            self.data.append(1.0 - float(pixel) / 255.0)
        self.height = rotated.shape[0]
        self.width = rotated.shape[1]
        logger.debug(f"height:{self.height}, width:{self.width}, origin:{self.origin}")

class MonteCarloLocalizer(Node):
    def __init__(self):
        super().__init__('monte_carlo_localizer')
        self.create_subscription(Odometry, '/odom', self.odometry_callback, 1)
        self.create_subscription(LaserScan, '/base_scan', self.scan_callback, 1)
        self.mcl_path = Path()
        self.odom_path = Path()
        self.mcl_path_pub = self.create_publisher(Path, '/mcl_path', 10)
        self.odom_path_pub = self.create_publisher(Path, '/odom_path', 10)
        self.particle_pub = self.create_publisher(PoseArray, '/particlecloud', 10)
        self.create_timer(0.1, self.timer_callback)
        self.particles: List[Particle] = []

        self.last_used_odom: Pose = None
        self.last_odom: Pose = None
        self.current_pose: Pose = None
        self.motion_model_cfg = None
        self.last_scan: LaserScan = None
        self.updating = False

        self.motion_model_cfg = {'alpha1': 0.001, 'alpha2': 0.001, 'alpha3': 0.01, 'alpha4':0.001}

        self.num_part = 1000

        self.map = Map('resource/epuck_world_map.yaml', self.get_logger())

        self.map_publisher = self.create_publisher(OccupancyGrid, '/map',
                qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            )
        )

        self.initialize_pose()
        self.initialize_particles_gaussian()

        self.tf_publisher = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom'
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        self.tf_publisher.sendTransform(tf)
        self.publish_map()

    def euler_to_quaternion(self, yaw, pitch, roll) -> Quaternion:
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

        return Quaternion(x=qx, y=qy, z=qz, w=qw)
    
    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z 

    def location_in_map_meter(self, x: float, y: float, map: Map):
        map_x = x - map.origin[0]
        map_y = y - map.origin[1]
        return (map_x, map_y) 
    
    def map_pose(self, pose: Pose):
        x = pose.position.x
        y = pose.position.y
        yaw = self.euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        map_x, map_y = self.location_in_map_meter(x, y, self.map)
        map_yaw = yaw
        return map_x, map_y, map_yaw

    def timer_callback(self):
        if self.last_used_odom and self.last_odom:
            self.update()
            self.publish_particles()
            self.last_used_odom = self.last_odom

    def odometry_callback(self, msg: Odometry):
        if not self.updating:
            self.last_odom = msg.pose.pose

    def scan_callback(self, msg: LaserScan):
        if not self.updating:
            self.last_scan = msg

    def initialize_pose(self):
        position = Point(x=0.0,
                         y=0.0,
                         z=0.0)
        orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
        self.current_pose = Pose(position=position,
                                  orientation=orientation)
        self.last_used_odom = self.current_pose
        self.last_odom = self.current_pose

    def initialize_particles_gaussian(self, pose: Pose = None, scale: float = 0.05):
        if pose is None:
            pose = self.current_pose

        x_list = list(np.random.normal(loc=pose.position.x, scale=scale, size=self._mcl_cfg['num_of_particles'] - 1))
        y_list = list(np.random.normal(loc=pose.position.y, scale=scale, size=self._mcl_cfg['num_of_particles'] - 1))
        current_yaw = self.euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        yaw_list = list(np.random.normal(loc=current_yaw, scale=0.01, size=self._mcl_cfg['num_of_particles'] - 1))

        initial_weight = 1.0 / float(self._mcl_cfg['num_of_particles'])

        for x, y, yaw in zip(x_list, y_list, yaw_list):
            position = Point(x=x, y=y, z=0.0)
            orientation = self.euler_to_quaternion(yaw, 0.0, 0.0)
            temp_pose = Pose(position=position, orientation=orientation)
            self.particles.append(Particle(temp_pose, initial_weight))

        self.particles.append(Particle(pose, initial_weight))

    def normalize_particles(self, particles: List[Particle], log):
        sum_of_weights = 0.0
        for particle in particles:
            sum_of_weights += particle.weight
        if sum_of_weights < 0.0001:
            return [Particle(particle.pose, 1.0 / len(particles)) for particle in particles]
        else:
            return [Particle(particle.pose, particle.weight / sum_of_weights) for particle in particles]

    def resample(self, particles: List[Particle]):
        weights = [particle.weight for particle in particles]
        cum_sums = np.cumsum(weights).tolist()
        n = 0
        new_samples = []

        initial_weight = 1.0 / float(self.mcl_cfg['num_of_particles'])
        while n < self.mcl_cfg['num_of_particles']:
            u = np.random.uniform(1e-6, 1, 1)[0]
            m = 0
            while cum_sums[m] < u:
                m += 1
            new_samples.append(Particle(particles[m].pose, initial_weight))
            n += 1

        return new_samples

    def best_particle(self, particles: List[Particle]):
        weight = 0.0
        pose = None
        for particle in particles:
            if particle.weight > weight:
                weight = particle.weight
                pose = particle.pose
        return pose, weight

    def pose_estimate(self, particles: List[Particle]):
        x = 0.0
        y = 0.0
        yaw = 0.0
        for particle in particles:
            x += particle.pose.position.x * particle.weight
            y += particle.pose.position.y * particle.weight
            _,_,yaw += self.euler_from_quaternion(particle.pose.orientation.x, particle.pose.orientation.y, particle.pose.orientation.z, particle.pose.orientation.w) * particle.weight
        position = Point(x=x, y=y, z=0.0)
        orientation = self.euler_to_quaternion(yaw=yaw, pitch=0.0, roll=0.0)
        return Pose(position=position, orientation=orientation)

    def should_resample(self, particles: List[Particle], resampling_threshold):
        sum_weights_squared = 0
        for particle in particles:
            sum_weights_squared += particle.weight ** 2

        return 1.0 / sum_weights_squared < resampling_threshold

    def generate_sample_index(particles: List[Particle]):
        weights = [particle.weight for particle in particles]
        cum_sums = np.cumsum(weights).tolist()
        m = 0
        u = np.random.uniform(1e-6, 1, 1)[0]
        while cum_sums[m] < u:
            m += 1

        return particles[m].pose

    def publish_particles(self):
        if len(self.particles) == 0:
            return
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for particle in self.particles:
            msg.poses.append(particle.pose)
        self.particle_pub.publish(msg)

    def publish_map(self):
        map = [-1] * self.map.width * self.map.height
        idx = 0
        for cell in self.map.data:
            map[idx] = int(cell * 100.0)
            idx += 1
        stamp = self.get_clock().now().to_msg()
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.info.resolution = self.map.resolution
        msg.info.width = self.map.width
        msg.info.height = self.map.width
        msg.info.origin.position.x = self.map.origin[0]
        msg.info.origin.position.y = self.map.origin[1]
        msg.data = map
        self.map_publisher.publish(msg)

    def publish_mcl_path(self, published_pose: Pose):
        stamp = self.get_clock().now().to_msg()
        self.mcl_path.header.frame_id = 'map'
        self.mcl_path.header.stamp = stamp
        pose = PoseStamped()
        pose.header = self.mcl_path.header
        pose.pose = published_pose
        self.mcl_path.poses.append(pose)
        self.mcl_path_pub.publish(self.mcl_path)

    def publish_odom_path(self, odom_pose: Pose):
        stamp = self.get_clock().now().to_msg()
        self.odom_path.header.frame_id = 'odom'
        self.odom_path.header.stamp = stamp
        pose = PoseStamped()
        pose.header = self.odom_path.header
        pose.pose = odom_pose
        self.odom_path.poses.append(pose)
        self.odom_path_pub.publish(self.odom_path)

    def update(self):
        if self.last_odom == self.last_used_odom:
            return

        self.updating = True
        new_particles = []

        for particle in self.particles:
            current_pose = particle.pose
            predicted_pose = motion_model.sample_odom_motion_model(current_pose,
                                                                   self.last_odom,
                                                                   self.last_used_odom,
                                                                   self.motion_model_cfg)
            prob = self.p_update(self.last_scan, predicted_pose)
            new_particles.append(Particle(predicted_pose, prob))

        self.particles.clear()
        self.particles = self.normalize_particles(new_particles, self.get_logger())
        self.current_pose = self.pose_estimate(self.particles)
        self.publish_mcl_path(self.current_pose)
        self.publish_odom_path(self.last_odom)

        if self.should_resample(self.particles, self.mcl_cfg['num_of_particles'] / 5.0):
            self.particles = self.resample(self.particles)

        self.updating = False

    def p_update(self, scan: LaserScan, pose: Pose):
        if scan is None:
            return 1.0

        prob = 1.0
        pose_in_map = self.map_pose(pose)

        for reading in scan:
            range_angle = pose_in_map.yaw + reading[0]
            range_x = pose_in_map.x + reading[1] * math.cos(range_angle)
            range_y = pose_in_map.y + reading[1] * math.sin(range_angle)

            if range_x >= self.map.width * self.map.resolution:
                range_x = (self.map.width - 1) * self.map.resolution
            if range_y >= self.map.height * self.map.resolution:
                range_y = (self.map.height - 1) * self.map.resolution

            expected = self.bresenham_line(int(pose_in_map.x / self.map.resolution),
                                            int(range_x / self.map.resolution),
                                            int(pose_in_map.y / self.map.resolution),
                                            int(range_y / self.map.resolution))
            real_reading = self._get_measurement_at(reading[0], scan)
            phit = self._get_prob(expected, real_reading, 0.1)
            prob *= phit

        return prob
    
    def bresenham_line(self, x0, x1, y0, y1):
        distance = 0.07
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            index = y0 * self.map.width + x0
            if index >= len(self.map.data) or index < -len(self.map.data):
                continue
            if self.map.data[index] >= self.map.occupied_thresh:
                delta_x = x0 - x1
                delta_y = y0 - y1
                distance = math.sqrt(delta_x ** 2 + delta_y ** 2) * self.map.resolution
                break
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
        return distance
    
    def _get_measurement_at(self, angle: float, scan: LaserScan):
        scan_angle = scan.angle_min
        for reading in scan.ranges:
            if np.isclose(scan_angle, angle, abs_tol=0.001):
                return reading
            scan_angle += scan.angle_increment

    def _get_prob(self, mean: float, reading: float, std_dev: float) -> tuple:
        if std_dev == 0.0:
            if np.isclose(reading, mean, abs_tol=0.001):
                return 1.0
            else:
                return 0.0
        probability = scipy.stats.norm(loc=mean, scale=std_dev).pdf(reading)
        if probability >= 1.0:
            probability = 0.999

        return probability

def main(args=None):
    rclpy.init(args=args)
    monte_carlo_localizer = MonteCarloLocalizer()
    rclpy.spin(monte_carlo_localizer)
    monte_carlo_localizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()