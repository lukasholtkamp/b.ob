#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
import math
import tf2_ros
import numpy as np
from tf_transformations import euler_from_quaternion


class LidarScanTriangleNode(Node):

    def __init__(self):
        super().__init__('lidar_scan_triangle_node')

        # Subscriber to /altered_scan topic (LaserScan)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/altered_scan',
            self.scan_callback,
            10
        )

        # Publisher for MarkerArray to visualize triangles
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        # Initialize tf2 buffer and listener to get the lidar_frame center
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_lidar_frame_transform(self):
        """Manually extract the translation and yaw angle from 'lidar_frame' to 'map'."""
        try:
            # Get the transform between 'map' and 'lidar_frame'
            transform = self.tf_buffer.lookup_transform('map', 'lidar_frame', rclpy.time.Time())

            # Extract the translation
            translation = transform.transform.translation
            translation_vec = np.array([translation.x, translation.y, translation.z])

            # Extract the rotation quaternion and convert it to euler angles (yaw is the Z-axis rotation)
            rotation = transform.transform.rotation
            quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
            _, _, yaw = euler_from_quaternion(quaternion)

            return translation_vec, yaw

        except Exception as e:
            self.get_logger().warn(f"Could not get transform for lidar_frame: {str(e)}")
            return None, None

    def rotate_point(self, point, yaw):
        """Rotate a point by the given yaw angle (Z-axis)."""
        # Create the rotation matrix for the yaw angle
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        rotation_matrix = np.array([[cos_yaw, -sin_yaw],
                                    [sin_yaw, cos_yaw]])

        # Apply the rotation to the point (assuming z=0, planar motion)
        point_vec = np.array([point.x, point.y])
        rotated_point_vec = np.dot(rotation_matrix, point_vec)

        # Return the rotated point
        return Point(x=rotated_point_vec[0], y=rotated_point_vec[1], z=0.0)

    def manually_transform_point(self, point, translation_vec, yaw):
        """Manually transform a point using translation and yaw rotation."""
        # Rotate the point by the yaw angle difference
        rotated_point = self.rotate_point(point, yaw)

        # Apply the translation to the rotated point
        transformed_point = Point(x=rotated_point.x + translation_vec[0],
                                  y=rotated_point.y + translation_vec[1],
                                  z=rotated_point.z + translation_vec[2])
        return transformed_point

    def scan_callback(self, msg):
        try:
            # Get the translation and yaw angle for manual transform
            translation_vec, yaw = self.get_lidar_frame_transform()

            # If the transform isn't available, skip this callback
            if translation_vec is None or yaw is None:
                return

            # Filters for range limits
            min_range = 0.1  # Minimum valid range value (ignore too close points)
            max_range = 10.0  # Maximum valid range value (ignore far points)

            # Process the scan data and manually transform all points first
            angle = msg.angle_min
            transformed_points = []
            for i in range(len(msg.ranges)):
                range_value = msg.ranges[i]

                # Skip invalid points (out of range or zero)
                if range_value < min_range or range_value > max_range:
                    angle += msg.angle_increment
                    continue

                # Calculate the position of the point in the lidar_frame
                x = range_value * math.cos(angle)
                y = range_value * math.sin(angle)
                point = Point(x=x, y=y, z=0.0)

                # Manually transform the point from lidar_frame to map frame
                transformed_point = self.manually_transform_point(point, translation_vec, yaw)

                transformed_points.append(transformed_point)

                # Increment the angle for the next scan point
                angle += msg.angle_increment

            # Create a marker after transforming all points
            marker_array = MarkerArray()

            # Create a single marker for all triangles
            marker = Marker()
            marker.header.frame_id = 'map'  # Set to 'map' frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'scan_triangles'
            marker.id = 0
            marker.type = Marker.TRIANGLE_LIST  # Use TRIANGLE_LIST to create filled triangles
            marker.action = Marker.ADD
            marker.scale.x = 1.0  # Required but not used for TRIANGLE_LIST
            marker.scale.y = 1.0
            marker.scale.z = 1.0

            # Set color for all triangles
            marker.color.r = 0.0  # All triangles will be red
            marker.color.g = 0.0  # All triangles will be green
            marker.color.b = 1.0  # (RGB) set to blue
            marker.color.a = 0.8  # Alpha (transparency)

            # Now create triangles between neighboring points
            lidar_frame_center = Point(x=translation_vec[0], y=translation_vec[1], z=translation_vec[2])
            for i in range(len(transformed_points) - 1):
                # Add the three vertices of the triangle (lidar center, transformed_point_1, transformed_point_2)
                marker.points.append(lidar_frame_center)
                marker.points.append(transformed_points[i])
                marker.points.append(transformed_points[i + 1])

            # Add this marker to the MarkerArray
            marker_array.markers.append(marker)

            # Publish the MarkerArray containing the filled triangles
            self.marker_pub.publish(marker_array)

        except Exception as e:
            self.get_logger().error(f"Error processing scan: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = LidarScanTriangleNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
