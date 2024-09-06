import numpy as np
from sklearn.cluster import DBSCAN
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math


class Detection(Node):

    def __init__(self):
        super().__init__("detection_node")

        # Subscriber for the scan
        self.scan_subscriber = self.create_subscription(
            LaserScan, "scan", self.scan_callback, 10
        )

        # Publisher for the filtered scan
        self.filtered_scan_publisher = self.create_publisher(
            LaserScan, "filtered_scan", 10
        )

        # Publisher for the marker array (to visualize clusters as circles)
        self.marker_publisher = self.create_publisher(
            MarkerArray, "cluster_markers", 10
        )

        # Keep track of the marker IDs
        self.marker_id = 0

    def scan_callback(self, scan_msg: LaserScan):
        # Extract ranges and angles (theta) from the LaserScan message
        original_ranges = np.array(scan_msg.ranges)
        num_ranges = len(original_ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, num_ranges)

        # Mask for valid (finite) range values
        valid_mask = np.isfinite(original_ranges)
        valid_angles = angles[valid_mask]
        valid_ranges = original_ranges[valid_mask]

        # Perform DBSCAN directly on theta (angles) and ranges
        polar_data = np.vstack((valid_angles, valid_ranges)).T  # Shape: (n, 2)

        # Perform DBSCAN clustering on the polar coordinates (theta, ranges)
        db = DBSCAN(eps=0.25, min_samples=3).fit(polar_data)
        labels = db.labels_  # Cluster labels

        # Initialize filtered ranges array with zeros (same size as original)
        filtered_ranges = np.zeros(num_ranges)

        # Prepare marker array to store circles for clusters
        marker_array = MarkerArray()

        # Process each cluster (excluding noise points with label -1)
        unique_labels = set(labels)
        for cluster_label in unique_labels:
            if cluster_label == -1:
                continue  # Skip noise points

            # Mask for points in the current cluster
            cluster_mask = labels == cluster_label
            cluster_angles = valid_angles[cluster_mask]
            cluster_ranges = valid_ranges[cluster_mask]

            # Filter clusters based on size (e.g., between 2 and 40 points)
            if 2 < len(cluster_ranges) < 40:
                # Find the mean of the cluster for marker positioning
                mean_x = np.mean(cluster_ranges * np.cos(cluster_angles))
                mean_y = np.mean(cluster_ranges * np.sin(cluster_angles))
                stddev_x = np.std(cluster_ranges * np.cos(cluster_angles))
                stddev_y = np.std(cluster_ranges * np.sin(cluster_angles))

                # print(stddev_x)
                # print(stddev_y, "\n")

                # Find the original indices for these points in the original scan
                for angle, range_value in zip(cluster_angles, cluster_ranges):
                    # Find the closest original angle to store the filtered value
                    original_index = np.argmin(np.abs(angles - angle))
                    filtered_ranges[original_index] = range_value  # Set filtered range

                if not (stddev_x > 0.15 or stddev_y > 0.19):
                    # Create a marker for this cluster
                    marker = Marker()
                    marker.header.frame_id = (
                        "lidar_frame"  # The frame in which the data is published
                    )
                    marker.type = Marker.CYLINDER  # Circle marker
                    marker.action = Marker.ADD
                    marker.id = self.marker_id
                    self.marker_id += 1

                    # Set marker position (convert polar coordinates back to Cartesian)
                    marker.pose.position.x = mean_x
                    marker.pose.position.y = mean_y
                    marker.pose.position.z = 0.0  # Flat on the ground (z = 0)

                    # Orientation (keep the marker upright)
                    marker.pose.orientation.x = 0.0
                    marker.pose.orientation.y = 0.0
                    marker.pose.orientation.z = 0.0
                    marker.pose.orientation.w = 1.0

                    # Set marker scale (diameter of the circle based on standard deviation)
                    marker.scale.x = float(
                        4 * max(stddev_x, stddev_y)
                    )  # Diameter along X
                    marker.scale.y = float(
                        4 * max(stddev_x, stddev_y)
                    )  # Diameter along Y
                    marker.scale.z = 0.1  # Height of the cylinder (thin circle)

                    # Set marker color
                    marker.color.r = 0.0
                    marker.color.g = 1.0  # Green
                    marker.color.b = 0.0
                    marker.color.a = 1.0  # Fully opaque

                    # Add this marker to the marker array
                    marker_array.markers.append(marker)

            # Update the marker array: clear old markers
            for marker in marker_array.markers:
                marker.lifetime = rclpy.time.Duration(seconds=0.2).to_msg()

        # Publish the filtered scan
        self.publish_filtered_scan(scan_msg, filtered_ranges)

        # Publish the marker array
        self.marker_publisher.publish(marker_array)

    def publish_filtered_scan(self, original_scan_msg, filtered_ranges):
        # Create a new LaserScan message with filtered ranges
        filtered_scan = LaserScan()
        filtered_scan.header = original_scan_msg.header
        filtered_scan.angle_min = original_scan_msg.angle_min
        filtered_scan.angle_max = original_scan_msg.angle_max
        filtered_scan.angle_increment = original_scan_msg.angle_increment
        filtered_scan.time_increment = original_scan_msg.time_increment
        filtered_scan.scan_time = original_scan_msg.scan_time
        filtered_scan.range_min = original_scan_msg.range_min
        filtered_scan.range_max = original_scan_msg.range_max

        # Set filtered ranges (all other ranges are 0)
        filtered_scan.ranges = filtered_ranges.tolist()

        # Publish the filtered scan
        self.filtered_scan_publisher.publish(filtered_scan)


def main(args=None):
    rclpy.init(args=args)
    node = Detection()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
