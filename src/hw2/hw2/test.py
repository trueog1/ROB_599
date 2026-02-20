import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs  # Essential for .transform()
from geometry_msgs.msg import PointStamped

class PointTransformer(Node):
    def __init__(self):
        super().__init__('point_transformer')
        
        # 1. Setup TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create a sample point in the "map" frame
        self.map_point = PointStamped()
        self.map_point.header.frame_id = "map"
        self.map_point.point.x = 1.5
        self.map_point.point.y = 2.0
        self.map_point.point.z = 0.0

        # Timer to transform every second
        self.timer = self.create_timer(1.0, self.transform_point)

    def transform_point(self):
        self.map_point.header.stamp = self.get_clock().now().to_msg()
        try:
            # 2. Transform the point to base_link
            transformed_point = self.tf_buffer.transform(
                self.map_point, 'base_link', timeout=rclpy.duration.Duration(seconds=0.1)
            )
            self.get_logger().info(
                f'Map Point: ({self.map_point.point.x}, {self.map_point.point.y}) -> '
                f'Robot Point: ({transformed_point.point.x:.2f}, {transformed_point.point.y:.2f})'
            )
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'Could not transform: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PointTransformer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# 2. Look up transform (map -> base_link)
transform = tf_buffer.lookup_transform("base_link", "map", rclpy.time.Time())

# 3. Transform the pose
pose_in_map = PoseStamped() # Assume this is populated
pose_in_base_link = tf2_geometry_msgs.do_transform_pose(pose_in_map, transform)'''