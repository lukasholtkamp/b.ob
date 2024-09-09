import numpy as np
from sklearn.cluster import DBSCAN
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math
import tf2_ros
import tf_transformations
import tf2_geometry_msgs  # This import enables the transformation of PoseStamped
from geometry_msgs.msg import TransformStamped, PoseStamped


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

        # TF Buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Keep track of the marker IDs
        self.marker_id = 0

        # Store previous obstacle means in odom frame (to detect motion)
        self.previous_means = {}

    def scan_callback(self, scan_msg: LaserScan):
        # Extract ranges and angles (theta) from the LaserScan message
        original_ranges = np.array(scan_msg.ranges)
        num_ranges = len(original_ranges)  # DEBUG : 360
        angles = np.linspace(
            scan_msg.angle_min, scan_msg.angle_max, num_ranges
        )  # DEBUG : Array from -3 to 3 , 360 sampels

        # Mask for valid (finite) range values
        valid_mask = np.isfinite(
            original_ranges
        )  # DEBUG PPA/Screenshots/09.09.2024/valid_mask_1-3
        valid_angles = angles[
            valid_mask
        ]  # DEBUG PPA/Screenshots/09.09.2024/valid_mask_1-3
        valid_ranges = original_ranges[
            valid_mask
        ]  # DEBUG PPA/Screenshots/09.09.2024/valid_mask_1-3

        # Perform DBSCAN directly on theta (angles) and ranges
        polar_data = np.vstack(
            (valid_angles, valid_ranges)
        ).T  # Shape: (n, 2) # DEBUG PPA/Screenshots/09.09.2024/vstack_1-3

        # # Or
        # raw_data = [valid_ranges, valid_angles]
        # polar_data = np.array(raw_data).T
        # polar_data[:, [0, 1]] = polar_data[:, [1, 0]]
        # print(polar_data)

        # Perform DBSCAN clustering on the polar coordinates (theta, ranges)
        db = DBSCAN(eps=0.25, min_samples=3).fit(polar_data)
        labels = (
            db.labels_
        )  # Cluster labels # DEBUG Array has a random numbers for the clusters (each cluster has a number)

        # Initialize filtered ranges array with zeros (same size as original)
        filtered_ranges = np.zeros(num_ranges)

        # Prepare marker array to store circles for clusters
        marker_array = MarkerArray()

        # Get the transformation from lidar_frame to odom frame # DEBUG PPA/Screenshots/09.09.2024/transform1_1-4
        try:
            transform = self.tf_buffer.lookup_transform(
                "odom", "lidar_frame", rclpy.time.Time()
            )
        except tf2_ros.LookupException as e:
            self.get_logger().warn("Transform not available: {}".format(e))
            return
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn("Transform not available: {}".format(e))
            return

        # Process each cluster (excluding noise points with label -1)
        unique_labels = set(labels)
        current_means = {}  # To store the current mean positions of obstacles
        for cluster_label in unique_labels:
            if cluster_label == -1:
                continue  # Skip noise points

            # Mask for points in the current cluster
            cluster_mask = (
                labels == cluster_label
            )  # DEBUG True, where the cluster_label(for example 2 obstacle and 1 wall -> 0,1 or 2) is in labels
            cluster_angles = valid_angles[
                cluster_mask
            ]  # DEBUG Store the angles, which are true in labels
            cluster_ranges = valid_ranges[
                cluster_mask
            ]  # DEBUG Store the ranges, which are true in labels

            # Filter clusters based on size (e.g., between 2 and 40 points)
            if 2 < len(cluster_ranges) < 40:
                # Find the mean of the cluster for marker positioning
                mean_x = np.mean(cluster_ranges * np.cos(cluster_angles))
                mean_y = np.mean(cluster_ranges * np.sin(cluster_angles))
                stddev_x = np.std(cluster_ranges * np.cos(cluster_angles))
                stddev_y = np.std(cluster_ranges * np.sin(cluster_angles))

                # Find the original indices for these points in the original scan
                for angle, range_value in zip(cluster_angles, cluster_ranges):
                    # Find the closest original angle to store the filtered value # DEBUG PPA/Screenshots/09.09.2024/filtered_ranges_1-5
                    original_index = np.argmin(np.abs(angles - angle))
                    filtered_ranges[original_index] = range_value  # Set filtered range

                # Convert mean position from lidar_frame to odom frame # DEBUG PPA/Screenshots/09.09.2024/transform2_1-4
                point_in_lidar = PoseStamped()
                point_in_lidar.header.frame_id = "lidar_frame"
                point_in_lidar.pose.position.x = mean_x
                point_in_lidar.pose.position.y = mean_y
                point_in_lidar.pose.position.z = 0.0
                point_in_lidar.pose.orientation.w = 1.0

                # Transform to odom frame
                try:
                    point_in_odom = self.tf_buffer.transform(
                        point_in_lidar, "odom", timeout=rclpy.time.Duration(seconds=0.1)
                    )
                    transformed_x = point_in_odom.pose.position.x
                    transformed_y = point_in_odom.pose.position.y

                    # Limit to 3 decimal places for accuracy
                    transformed_x = round(transformed_x, 3)
                    transformed_y = round(transformed_y, 3)

                    # # Log the transformed position
                    # self.get_logger().info(
                    #     f"Obstacle in odom frame: x={transformed_x}, y={transformed_y}"
                    # )

                    # Store the current mean position
                    current_means[cluster_label] = (transformed_x, transformed_y)
                except tf2_ros.TransformException as e:
                    self.get_logger().warn(f"Failed to transform point: {e}")
                    continue

                # Check for movement (if obstacle moved compared to the previous frame) # DEBUG PPA/Screenshots/09.09.2024/dynamic_obstacle_1-3
                dynamic_obstacle = False
                if cluster_label in self.previous_means:
                    prev_x, prev_y = self.previous_means[cluster_label]
                    # Compare the mean positions of x and y
                    dist_moved = math.sqrt(
                        (transformed_x - prev_x) ** 2 + (transformed_y - prev_y) ** 2
                    )
                    if dist_moved > 0.035:  # Threshold for detecting movement
                        dynamic_obstacle = True

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
                    if dynamic_obstacle:
                        marker.color.r = 1.0  # Red for dynamic obstacles
                        marker.color.g = 0.0
                        marker.color.b = 0.0
                        marker.color.a = 1.0  # Fully opaque
                    else:
                        marker.color.r = 0.0  # Green for static obstacles
                        marker.color.g = 1.0
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

        # Store the current mean positions as previous for the next cycle
        self.previous_means = current_means

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
