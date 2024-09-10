import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped

class RayCastNode(Node):
    def __init__(self):
        super().__init__('ray_cast_node')

        # Subscriber to the LiDAR scan
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',  # LiDAR scan topic
            self.lidar_callback,
            10)

        # Subscriber to the costmap
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',  # Costmap topic
            self.costmap_callback,
            10)

        # Publisher for the altered LiDAR scan
        self.scan_pub = self.create_publisher(LaserScan, '/altered_scan', 10)

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.costmap_data = None
        self.map_resolution = None
        self.map_origin = None

    def costmap_callback(self, msg: OccupancyGrid):
        self.costmap_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]

    def lidar_callback(self, msg: LaserScan):
        if self.costmap_data is None:
            return

        # Get the transform for the lidar_frame to the map frame
        try:
            transform = self.tf_buffer.lookup_transform('map', 'lidar_frame', rclpy.time.Time())

            # Perform ray casting using the transformed LiDAR frame position in the map frame
            altered_scan = self.perform_ray_casting(msg, transform)

            # Publish the altered scan
            self.scan_pub.publish(altered_scan)

        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn('Transform not available between map and lidar_frame')

    def perform_ray_casting(self, scan: LaserScan, transform: TransformStamped) -> LaserScan:
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        ranges = np.array(scan.ranges)

        # Replace invalid ranges with max range
        ranges = np.where(np.isfinite(ranges), ranges, scan.range_max)

        # Get LiDAR's position and yaw in the map frame
        lidar_origin_x = transform.transform.translation.x
        lidar_origin_y = transform.transform.translation.y
        lidar_origin = [lidar_origin_x, lidar_origin_y]

        # Get the yaw from the lidar_frame to the map frame
        lidar_yaw = self.get_yaw_from_quaternion(transform.transform.rotation)

        # Adjust angles by adding the yaw to account for LiDAR rotation relative to the map frame
        adjusted_angles = angles + lidar_yaw  # Apply correct rotation adjustment

        # Perform the ray-casting logic with adjusted angles
        truncated_ranges = ray_cast_bresenham_optimized(
            lidar_origin, adjusted_angles, ranges, scan.range_max, self.costmap_data, self.map_origin, self.map_resolution)

        # Create the altered scan message
        altered_scan = LaserScan()
        altered_scan.header = scan.header
        altered_scan.header.frame_id = 'lidar_frame'  # Keep the frame_id as lidar_frame
        altered_scan.angle_min = scan.angle_min
        altered_scan.angle_max = scan.angle_max
        altered_scan.angle_increment = scan.angle_increment
        altered_scan.time_increment = scan.time_increment
        altered_scan.scan_time = scan.scan_time
        altered_scan.range_min = scan.range_min
        altered_scan.range_max = scan.range_max
        altered_scan.ranges = truncated_ranges.tolist()
        altered_scan.intensities = scan.intensities

        return altered_scan

    def get_yaw_from_quaternion(self, quaternion):
        """Convert quaternion to yaw (rotation around Z-axis)"""
        q_x = quaternion.x
        q_y = quaternion.y
        q_z = quaternion.z
        q_w = quaternion.w
        yaw = np.arctan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z))
        return yaw

# Optimized ray-casting function with debugging and boundary checks
def ray_cast_bresenham_optimized(lidar_origin, angles, ranges, range_max, costmap, map_origin, map_resolution):
    lidar_grid_x = np.floor((lidar_origin[0] - map_origin[0]) / map_resolution).astype(int)
    lidar_grid_y = np.floor((lidar_origin[1] - map_origin[1]) / map_resolution).astype(int)

    end_x = lidar_origin[0] + ranges * np.cos(angles)
    end_y = lidar_origin[1] + ranges * np.sin(angles)

    # Ensure points are clipped within the map boundary
    end_x = np.clip(end_x, map_origin[0], map_origin[0] + costmap.shape[1] * map_resolution)
    end_y = np.clip(end_y, map_origin[1], map_origin[1] + costmap.shape[0] * map_resolution)

    end_grid_x = np.floor((end_x - map_origin[0]) / map_resolution).astype(int)
    end_grid_y = np.floor((end_y - map_origin[1]) / map_resolution).astype(int)

    truncated_ranges = np.full_like(ranges, range_max)

    for i in range(len(angles)):
        if not is_in_grid(end_grid_x[i], end_grid_y[i], costmap.shape):
            truncated_ranges[i] = ranges[i]  # If point is out of bounds, keep original range
            continue

        points = bresenham(lidar_grid_x, lidar_grid_y, end_grid_x[i], end_grid_y[i])

        for grid_x, grid_y in points:
            if not is_in_grid(grid_x, grid_y, costmap.shape):
                break
            if costmap[grid_y, grid_x] > 50:  # Occupied cell
                hit_x, hit_y = grid_to_world(grid_x, grid_y, map_origin, map_resolution)
                distance = np.sqrt((hit_x - lidar_origin[0]) ** 2 + (hit_y - lidar_origin[1]) ** 2)
                truncated_ranges[i] = min(truncated_ranges[i], distance)
                break
        else:
            # If no obstacle is hit, keep the original range
            truncated_ranges[i] = ranges[i]

    return truncated_ranges

# Bresenham's line algorithm (unchanged)
def bresenham(x1, y1, x2, y2):
    points = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    while True:
        points.append((x1, y1))
        if x1 == x2 and y1 == y2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy

    return points

# Helper functions (unchanged)
def is_in_grid(grid_x, grid_y, shape):
    return 0 <= grid_x < shape[1] and 0 <= grid_y < shape[0]

def grid_to_world(grid_x, grid_y, map_origin, map_resolution):
    x = map_origin[0] + grid_x * map_resolution + map_resolution / 2
    y = map_origin[1] + grid_y * map_resolution + map_resolution / 2
    return x, y

def main(args=None):
    rclpy.init(args=args)
    node = RayCastNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
