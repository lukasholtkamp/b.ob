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
import tf2_geometry_msgs  # Enables the transformation of PoseStamped
from geometry_msgs.msg import TransformStamped, PoseStamped

# from pykalman import KalmanFilter
import time


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

        # Cluster tracking variables
        self.cluster_tracker = {}  # maps cluster IDs to cluster data
        self.next_cluster_id = 0  # to assign new IDs
        self.max_cluster_distance = (
            0.5  # maximum allowed distance to match clusters (tune as needed)
        )
        self.max_missing_frames = 5  # Maximum frames to keep a missing cluster
        self.frame_counter = 0  # Frame counter

        # Set to keep track of active marker IDs
        self.active_marker_ids = set()

    def scan_callback(self, scan_msg: LaserScan):
        # Increment frame counter
        self.frame_counter += 1

        # Extract ranges and angles (theta) from the LaserScan message
        original_ranges = np.array(scan_msg.ranges)
        num_ranges = len(original_ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, num_ranges)

        # Mask for valid (finite) range values
        valid_mask = np.isfinite(original_ranges)
        valid_angles = angles[valid_mask]
        valid_ranges = original_ranges[valid_mask]

        # Perform DBSCAN directly on theta (angles) and ranges
        polar_data = np.vstack((valid_angles, valid_ranges)).T

        # Perform DBSCAN clustering on the polar coordinates (theta, ranges)
        db = DBSCAN(eps=0.45, min_samples=2).fit(polar_data)
        labels = db.labels_

        # Initialize filtered ranges array with zeros (same size as original)
        filtered_ranges = np.zeros(num_ranges)

        # Prepare marker array to store circles for clusters
        marker_array = MarkerArray()

        # Get the transformation from lidar_frame to odom frame
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
        current_clusters = []

        for cluster_label in unique_labels:
            if cluster_label == -1:
                continue  # Skip noise points

            # Mask for points in the current cluster
            cluster_mask = labels == cluster_label
            cluster_angles = valid_angles[cluster_mask]
            cluster_ranges = valid_ranges[cluster_mask]

            # Filter clusters based on size (e.g., between 2 and 35 points)
            if 2 < len(cluster_ranges) < 25:
                # Find the mean of the cluster for marker positioning
                mean_x = np.mean(cluster_ranges * np.cos(cluster_angles))
                mean_y = np.mean(cluster_ranges * np.sin(cluster_angles))
                stddev_x = np.std(cluster_ranges * np.cos(cluster_angles))
                stddev_y = np.std(cluster_ranges * np.sin(cluster_angles))

                # Find the original indices for these points in the original scan
                for angle, range_value in zip(cluster_angles, cluster_ranges):
                    original_index = np.argmin(np.abs(angles - angle))
                    filtered_ranges[original_index] = range_value

                # Convert mean position from lidar_frame to odom frame
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

                    # Store the current cluster data
                    current_cluster = {
                        "mean_x": transformed_x,
                        "mean_y": transformed_y,
                        "stddev_x": stddev_x,
                        "stddev_y": stddev_y,
                        "cluster_label": cluster_label,  # The label assigned by DBSCAN (for reference)
                    }

                    current_clusters.append(current_cluster)

                except tf2_ros.TransformException as e:
                    self.get_logger().warn(f"Failed to transform point: {e}")
                    continue

        # Perform data association between current clusters and previous clusters
        matched_prev_ids = set()

        for current_cluster in current_clusters:
            current_x = current_cluster["mean_x"]
            current_y = current_cluster["mean_y"]

            min_distance = float("inf")
            matched_id = None

            # Loop over previous clusters
            for cluster_id, cluster_data in self.cluster_tracker.items():
                if cluster_id in matched_prev_ids:
                    continue  # This previous cluster is already matched

                prev_x = cluster_data["mean_x"]
                prev_y = cluster_data["mean_y"]

                distance = math.hypot(current_x - prev_x, current_y - prev_y)

                if distance < min_distance:
                    min_distance = distance
                    matched_id = cluster_id

            if min_distance < self.max_cluster_distance:
                # Match found
                current_cluster["cluster_id"] = matched_id
                matched_prev_ids.add(matched_id)

                # Update cluster data in tracker
                self.cluster_tracker[matched_id]["mean_x"] = current_x
                self.cluster_tracker[matched_id]["mean_y"] = current_y
                self.cluster_tracker[matched_id]["stddev_x"] = current_cluster[
                    "stddev_x"
                ]
                self.cluster_tracker[matched_id]["stddev_y"] = current_cluster[
                    "stddev_y"
                ]
                self.cluster_tracker[matched_id]["last_seen"] = self.frame_counter
                self.cluster_tracker[matched_id][
                    "missing_frames"
                ] = 0  # Reset missing frames

            else:
                # No match found, assign new cluster ID
                new_cluster_id = self.next_cluster_id
                self.next_cluster_id += 1

                current_cluster["cluster_id"] = new_cluster_id

                # Add to cluster tracker
                self.cluster_tracker[new_cluster_id] = {
                    "mean_x": current_x,
                    "mean_y": current_y,
                    "stddev_x": current_cluster["stddev_x"],
                    "stddev_y": current_cluster["stddev_y"],
                    "last_seen": self.frame_counter,
                    "missing_frames": 0,
                    "prev_mean_x": current_x,
                    "prev_mean_y": current_y,
                }

        # Update missing frames for unmatched previous clusters
        for cluster_id, cluster_data in self.cluster_tracker.items():
            if cluster_id not in matched_prev_ids:
                cluster_data["missing_frames"] += 1

        # Remove clusters that have been missing for too long
        clusters_to_remove = []
        for cluster_id, cluster_data in self.cluster_tracker.items():
            if cluster_data["missing_frames"] > self.max_missing_frames:
                clusters_to_remove.append(cluster_id)

        for cluster_id in clusters_to_remove:
            del self.cluster_tracker[cluster_id]

            # Create a DELETE marker
            delete_marker = Marker()
            delete_marker.header.frame_id = "odom"
            delete_marker.action = Marker.DELETE
            delete_marker.id = cluster_id
            marker_array.markers.append(delete_marker)

            # Remove from active_marker_ids
            self.active_marker_ids.discard(cluster_id)

        # Now process current clusters for visualization and movement detection
        for current_cluster in current_clusters:
            cluster_id = current_cluster["cluster_id"]
            current_x = current_cluster["mean_x"]
            current_y = current_cluster["mean_y"]
            stddev_x = current_cluster["stddev_x"]
            stddev_y = current_cluster["stddev_y"]

            # Retrieve previous position from cluster tracker
            cluster_data = self.cluster_tracker[cluster_id]

            # Check for movement
            dynamic_obstacle = False
            prev_x = cluster_data.get("prev_mean_x", current_x)
            prev_y = cluster_data.get("prev_mean_y", current_y)

            diff_x = abs(current_x - prev_x)
            diff_y = abs(current_y - prev_y)

            movement_threshold = 0.05  # Set appropriate threshold

            if diff_x > movement_threshold or diff_y > movement_threshold:
                dynamic_obstacle = True

            # Update previous position in cluster_data
            cluster_data["prev_mean_x"] = current_x
            cluster_data["prev_mean_y"] = current_y

            # Filter out noise based on standard deviations
            if not (stddev_x > 0.15 or stddev_y > 0.20):
                # Create a marker for this cluster
                marker = Marker()
                marker.header.frame_id = "odom"
                marker.type = Marker.CYLINDER  # Circle marker
                marker.action = Marker.ADD
                marker.id = cluster_id  # Use cluster_id as marker id

                # Set marker position
                marker.pose.position.x = current_x
                marker.pose.position.y = current_y
                marker.pose.position.z = 0.0

                # Orientation (keep the marker upright)
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0

                # Set marker scale
                marker.scale.x = float(4 * max(stddev_x, stddev_y))
                marker.scale.y = float(4 * max(stddev_x, stddev_y))
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

                # Keep track of active marker IDs
                self.active_marker_ids.add(cluster_id)

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
    detection_node = Detection()
    try:
        rclpy.spin(detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        detection_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
