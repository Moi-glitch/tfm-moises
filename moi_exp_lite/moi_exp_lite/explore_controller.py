#!/usr/bin/env python3
import math
import os
from collections import deque
import yaml  # Import the YAML module

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy  # Import QoS settings
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool, String
from sensor_msgs.msg import LaserScan  # For subscribing to /scan
from visualization_msgs.msg import Marker  # Import for RViz markers
from geometry_msgs.msg import PointStamped
import time
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from nav_msgs.msg import OccupancyGrid

class ExploreController(Node):
    def __init__(self):
        super().__init__('explore_controller')


        # quitar si no va
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value


        # Declare parameters
        self.declare_parameter('name_object_thr', 0.05)
        self.declare_parameter('I_min', 200.0)
        self.declare_parameter('I_max', 6000.0)
        self.declare_parameter('bw_min', 0.01)
        self.declare_parameter('bw_max', 0.45)
        self.declare_parameter('cluster_radius', 10)

        # Default path for storing detected objects
        default_yaml = os.path.join(
            get_package_share_directory('moi_exp_lite'),
            'detected_objects.yaml')
        self.declare_parameter('yaml_file_path', default_yaml)

        # Retrieve parameters
        self.name_object_thr = self.get_parameter('name_object_thr').get_parameter_value().double_value
        self.I_min = self.get_parameter('I_min').get_parameter_value().double_value
        self.I_max = self.get_parameter('I_max').get_parameter_value().double_value
        self.bw_min = self.get_parameter('bw_min').get_parameter_value().double_value
        self.bw_max = self.get_parameter('bw_max').get_parameter_value().double_value
        self.cluster_radius = self.get_parameter('cluster_radius').get_parameter_value().integer_value
        self.yaml_file_path = self.get_parameter('yaml_file_path').get_parameter_value().string_value

        # Resolve path relative to package share if needed
        if not os.path.isabs(self.yaml_file_path):
            pkg_share = get_package_share_directory('moi_exp_lite')
            self.yaml_file_path = os.path.join(pkg_share, self.yaml_file_path)

        # Publisher to resume/stop exploration
        self.resume_pub = self.create_publisher(Bool, '/explore/resume', 10)

        # Publisher for RViz markers
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        # Publisher that notifies when a red object has been marked
        self.mark_pub = self.create_publisher(Bool, '/object_marked', 10)

        # Add a class attribute to track marker IDs
        self.marker_id = 0  # Initialize marker ID

        # Subscribe to color detections
        self.color_sub = self.create_subscription(
            String,
            '/color_detection',
            self.object_detection,  # Updated method name
            10
        )

        # Subscribe to the costmap for occupancy grid updates
        self.costmap_sub = self.create_subscription(   
            OccupancyGrid,
            '/move_base/global_costmap/costmap',
            self.costmap_callback,
            10
        )
        self.latest_costmap = None  # Initialize costmap attribute

        # Subscribe to LiDAR scans with Best Effort QoS and buffer the last 20 messages
        self.scan_buffer = deque(maxlen=20)
        scan_qos = QoSProfile(depth=10)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            scan_qos
        )

        # Timer to keep exploration running if we haven't stopped yet
        self.stop_sent = False
        self.timer = self.create_timer(1.0, self.send_resume_if_not_stopped)

        # List to store received color-detection data
        self.detected_data = []

        # TF buffer and listener for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # List to store global detections
        self.global_detections = []

    def get_robot_global_coordinates(self):
        """
        Get the global coordinates of the robot in the map frame.
        Returns a tuple (x_map, y_map) representing the robot's position in the map frame.
        """
        try:
            # Lookup the transform from the map frame to the robot's base_link frame
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            
            # Extract the translation (x, y) from the transform
            x_tb3 = transform.transform.translation.x
            y_tb3 = transform.transform.translation.y

            #self.get_logger().info(f"Robot global coordinates: x_map={x_map:.2f}, y_map={y_map:.2f}")
            return x_tb3, y_tb3

        except tf2_ros.LookupException:
            self.get_logger().error("TF lookup failed: Could not find transform from 'map' to 'base_link'.")
        except tf2_ros.ConnectivityException:
            self.get_logger().error("TF connectivity error: Could not connect to TF tree.")
        except tf2_ros.ExtrapolationException:
            self.get_logger().error("TF extrapolation error: Transform is not available for the requested time.")

        # Return None if the transform fails
        return None, None

    def send_resume_if_not_stopped(self):
        """Periodically publish a resume command unless a stop was sent."""
        if not self.stop_sent:
            self.resume_pub.publish(Bool(data=True))


    def costmap_callback(self, msg: OccupancyGrid):
        """
        Callback to receive the latest costmap and store it.
        This can be used for further processing or analysis.
        """
        self.latest_costmap = msg
        #self.get_logger().info(f"Received costmap with resolution {msg.info.resolution} m/pixel and size {msg.info.width}x{msg.info.height} pixels.")

    def costmap_to_binary_grid(self, costmap: OccupancyGrid, threshold=99):
        """Convert the costmap to a 2D grid of 1s (occupied) and 0s (free)."""
        width = costmap.info.width
        height = costmap.info.height
        grid = [[0 for _ in range(width)] for _ in range(height)]
        for i, val in enumerate(costmap.data):
            if val >= threshold:
                x = i % width
                y = i // width
                grid[y][x] = 1
        return grid

    def world_to_grid(self, x, y, costmap: OccupancyGrid):
        """Convert world coordinates to grid coordinates."""
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y
        resolution = costmap.info.resolution
        gx = int((x - origin_x) / resolution)
        gy = int((y - origin_y) / resolution)
        return gx, gy

    def grid_to_world(self, gx, gy, costmap: OccupancyGrid):
        """Convert grid coordinates back to world coordinates."""
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y
        resolution = costmap.info.resolution
        x = gx * resolution + origin_x + resolution / 2.0
        y = gy * resolution + origin_y + resolution / 2.0
        return x, y

    def _extract_cluster(self, grid, gx, gy):
        """Simple flood fill to extract a connected cluster of occupied cells."""
        width = len(grid[0])
        height = len(grid)
        if gx < 0 or gy < 0 or gx >= width or gy >= height:
            return []
        if grid[gy][gx] == 0:
            return []

        stack = [(gx, gy)]
        visited = set()
        cluster = []
        while stack:
            x, y = stack.pop()
            if (x, y) in visited:
                continue
            visited.add((x, y))
            if 0 <= x < width and 0 <= y < height and grid[y][x] == 1:
                cluster.append((x, y))
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    stack.append((x + dx, y + dy))
        return cluster

    def _cluster_center(self, cluster, costmap: OccupancyGrid):
        """Return the world coordinates for the center of a cluster."""
        if not cluster:
            return None, None
        xs = [c[0] for c in cluster]
        ys = [c[1] for c in cluster]
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)
        return self.grid_to_world(int(round(cx)), int(round(cy)), costmap)

    def get_obstacle_points(self,costmap:OccupancyGrid,threshold=99):
        """
        Extract obstacle points from the costmap based on a threshold.
        Returns a list of (x, y) coordinates of obstacles.
        """
        if costmap is None:
            self.get_logger().warn("No costmap available.")
            return []

        obstacle_points = []
        width = costmap.info.width
        height = costmap.info.height
        resolution = costmap.info.resolution

        for i in range(width * height):
            if costmap.data[i] >= threshold:
                # Convert index to (x, y) coordinates
                x = (i % width) * resolution + costmap.info.origin.position.x
                y = (i // width) * resolution + costmap.info.origin.position.y
                obstacle_points.append((x, y))
        return obstacle_points


    def is_duplicate_detection(self, x_map, y_map, x_threshold=0.5, y_threshold=0.5):
        """
        Check if the given coordinates (x_map, y_map) are already in the vicinity of any stored global detections.
        A detection is considered duplicate if the new coordinates fall within a 1-meter window (both x and y)
        of any stored coordinates.
        """
        self.get_logger().info(f"Checking duplicate detection for x_map={x_map}, y_map={y_map}")
        for detection in self.global_detections:
            stored_x = detection["x_map"]
            stored_y = detection["y_map"]

            # Check if the new coordinates are within the vicinity of the stored coordinates
            if abs(x_map - stored_x) <= x_threshold and abs(y_map - stored_y) <= y_threshold:
                self.get_logger().info(
                    f"Duplicate detection found: x_map={x_map:.2f}, y_map={y_map:.2f} is within the vicinity of stored detection x_map={stored_x:.2f}, y_map={stored_y:.2f}."
                )
                return True
        return False

    def save_detections_to_yaml(self):
        """
        Save the global detections to a YAML file.
        """
        try:
            # Convert NumPy objects to standard Python types
            detections_to_save = []
            for detection in self.global_detections:
                detections_to_save.append({
                    "color": detection["color"],
                    "timestamp": detection["timestamp"],
                    "x_map": float(detection["x_map"]),  # Convert to float
                    "y_map": float(detection["y_map"])   # Convert to float
                })

            with open(self.yaml_file_path, 'w') as yaml_file:
                yaml.dump(detections_to_save, yaml_file, default_flow_style=False)
            self.get_logger().info(f"Global detections saved to {self.yaml_file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save detections to YAML file: {e}")

    def marker_creation(self, final_angle, t_detect, distance):
        # Define the maximum distance for marking
        max_distance = 2.0  # Limit to 2 meters

        # Check if the object is within the allowed distance
        if distance > max_distance:
            self.get_logger().info(
                f"Object at distance {distance:.2f} m exceeds the maximum allowed distance of {max_distance} m. Marker not created."
            )
            return

        # Compute (x, y) in robot frame using the final angle
        x_robot = distance * math.cos(final_angle)
        y_robot = distance * math.sin(final_angle)

        self.get_logger().info(f"Computed object coordinates in robot frame: x={x_robot:.2f}, y={y_robot:.2f}")

        # Wrap the local point in a PointStamped living in base_link
        pt_base = PointStamped()
        pt_base.header.stamp = self.get_clock().now().to_msg()
        pt_base.header.frame_id = "base_link"
        pt_base.point.x, pt_base.point.y, pt_base.point.z = x_robot, y_robot, 0.0

        try:
            self.get_logger().info(f"Attempting TF transformation for x_robot={x_robot}, y_robot={y_robot}")
            for _ in range(3):  # Retry up to 3 times
                try:
                    transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
                    break
                except tf2_ros.LookupException:
                    self.get_logger().warn("TF lookup failed, retrying...")
                    time.sleep(0.1)
            else:
                self.get_logger().error("TF lookup failed after retries.")
                return

            pt_map = do_transform_point(pt_base, transform)
            x_map, y_map = pt_map.point.x, pt_map.point.y
            self.get_logger().info(f"TF transformation successful: x_map={x_map}, y_map={y_map}")
        except (tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        # Optionally adjust coordinates using the costmap
        obstacle_points = self.get_obstacle_points(self.latest_costmap)
        x_map, y_map = self.correction_via_costmap(x_map, y_map, obstacle_points)

        # Check for duplicate detection after correction
        if self.is_duplicate_detection(x_map, y_map):
            self.get_logger().info(
                f"Duplicate detection discarded: x_map={x_map:.2f}, y_map={y_map:.2f}."
            )
            return

        # Create and publish the RViz marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "red_object"
        marker.id = self.marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x_map
        marker.pose.position.y = y_map
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

        self.get_logger().info(
            f'🔴 Red detected at angle={final_angle * (180.0 / math.pi):.2f}°, timestamp={t_detect}. Marker placed at (x_map={x_map:.2f}, y_map={y_map:.2f}). Marker ID: {self.marker_id}.'
        )

        # Add the detection to the global detections list
        self.global_detections.append({
            "color": "red",
            "x_map": x_map,
            "y_map": y_map,
            "timestamp": self.get_clock().now().nanoseconds
        })

        # Save the updated detections to the YAML file
        self.save_detections_to_yaml()

        # Notify other components that a red object has been marked
        self.mark_pub.publish(Bool(data=True))

        # Increment the marker ID
        self.marker_id += 1

        # Limit the size of the global detections list
        if len(self.global_detections) > 100:
            self.global_detections.pop(0)

    def correction_via_costmap(self, x_map, y_map, obstacle_points, radius=0.5):
        """
        Correct the given (x_map, y_map) coordinates by snapping to the nearest obstacle point
        in the costmap or finding an isolated object along the line from the robot to the marker.
        """
        if self.latest_costmap is None:
            self.get_logger().warn("No costmap available for correction.")
            return x_map, y_map

        if not obstacle_points:
            self.get_logger().warn("No obstacles found in the costmap.")
            return x_map, y_map

        costmap = self.latest_costmap
        grid = self.costmap_to_binary_grid(costmap)

        gx, gy = self.world_to_grid(x_map, y_map, costmap)
        width = len(grid[0])
        height = len(grid)

        # Helper to validate a cluster as an isolated object
        def valid_cluster(cl):
            if not cl:
                return False
            xs = [c[0] for c in cl]
            ys = [c[1] for c in cl]
            w = max(xs) - min(xs) + 1
            h = max(ys) - min(ys) + 1
            area = len(cl)
            # Heuristic: discard very large clusters (likely walls)
            return area < 400 and w < 20 and h < 20

        if 0 <= gx < width and 0 <= gy < height and grid[gy][gx] == 1:
            cluster = self._extract_cluster(grid, gx, gy)
            if valid_cluster(cluster):
                cx, cy = self._cluster_center(cluster, costmap)
                self.get_logger().info(f"Marker inside obstacle, snapping to object center ({cx}, {cy})")
                return cx, cy

        robot_x, robot_y = self.get_robot_global_coordinates()
        if robot_x is None or robot_y is None:
            self.get_logger().warn("Could not get robot global coordinates. Returning original coordinates.")
            return x_map, y_map

        steps = 100
        for step in range(1, steps + 1):
            t = step / steps
            wx = robot_x + t * (x_map - robot_x)
            wy = robot_y + t * (y_map - robot_y)
            gx, gy = self.world_to_grid(wx, wy, costmap)
            if 0 <= gx < width and 0 <= gy < height and grid[gy][gx] == 1:
                cluster = self._extract_cluster(grid, gx, gy)
                if valid_cluster(cluster):
                    cx, cy = self._cluster_center(cluster, costmap)
                    self.get_logger().info(f"Adjusted marker along line to ({cx}, {cy})")
                    return cx, cy

        self.get_logger().info("No suitable obstacle found; returning original coordinates.")
        return x_map, y_map

    def object_detection(self, msg: String):
        try:
            self.get_logger().info("Debug: Entering object_detection method")
            self.get_logger().info(f"Debug: Received message: {msg.data}")
            # Parse the incoming message
            data_parts = msg.data.split(',')
            color = data_parts[0].strip()
            angle_rad = float(data_parts[1].split('=')[1])
            t_detect = float(data_parts[2].split('=')[1])  # Use floating-point timestamp

            self.detected_data.append({
                'color': color,
                'angle': angle_rad,
                'timestamp': t_detect
            })

            # FIRST OBJECT CONTACT --------------------------------------------------
            if color.lower() == 'red' and not self.stop_sent:
                # Find the scan whose header.stamp is nearest to t_detect
                closest_scan = None
                best_diff = float('inf')
                for scan_msg in self.scan_buffer:
                    t_scan = scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec * 1e-9
                    d = abs(t_scan - t_detect)
                    self.get_logger().debug(f"Timestamp difference: {d:.6f} seconds")
                    if d < best_diff:
                        best_diff = d
                        closest_scan = scan_msg

                max_diff = 0.3  # Allow up to x seconds difference
                if best_diff > max_diff:
                    self.get_logger().warn(f"No LiDAR scan found within {max_diff} seconds of the detection timestamp.")
                    return

                if closest_scan is None:
                    closest_scan = self.scan_buffer[-1]  # Use the most recent scan
                    self.get_logger().warn("No close match found; using the most recent scan as fallback.")

                # Wrap angle_rad into [angle_min, angle_max]
                wrapped_angle = (-angle_rad) % (2 * math.pi)
                epsilon = 1e-6
                if wrapped_angle > closest_scan.angle_max - epsilon:
                   wrapped_angle = 0.0   
                if wrapped_angle < closest_scan.angle_min or wrapped_angle > closest_scan.angle_max:
                    self.get_logger().warn(f"Angle {wrapped_angle:.2f} rad is outside the LiDAR scan range [{closest_scan.angle_min:.2f}, {closest_scan.angle_max:.2f}].")
                    return

                # Compute index i for the wrapped angle
                i_wrapped = int(round((wrapped_angle - closest_scan.angle_min) / closest_scan.angle_increment))
                i_wrapped = max(0, min(i_wrapped, len(closest_scan.ranges) - 1))  # Ensure index is within bounds

                # Debug logs for computed values
                self.get_logger().info(f"Wrapped angle: {wrapped_angle:.2f} rad --- {wrapped_angle * (180.0 / math.pi):.2f}°")
                self.get_logger().info(f"Computed wrapped index: {i_wrapped}, angle_min: {closest_scan.angle_min:.2f}, angle_max: {closest_scan.angle_max:.2f}, angle_increment: {closest_scan.angle_increment:.6f}")
                self.get_logger().info(f"Distance at wrapped index {i_wrapped}: {closest_scan.ranges[i_wrapped]:.2f}")

                # Get the distance at the computed index
                distance = closest_scan.ranges[i_wrapped]
                if math.isinf(distance) or math.isnan(distance) or distance <= 0.0:
                    self.get_logger().warn("LiDAR return invalid at that angle; optionally try the next closest scan.")
                    return

                # Check if the object is within 1.9 meters
                if distance > 1.5:
                    self.get_logger().info(f"Object detected at {distance:.2f} m, which exceeds the 1.9 m threshold. Clustering will not start.")
                    return


                # OBJECT DETECTION USING CLUSTERING ----------------------------------------------
                import numpy as np
                from scipy.ndimage import label


                # Parameters
                same_object_thr = self.name_object_thr  # Minimum distance to consider a valid object
                I_min = self.I_min  # Minimum intensity threshold
                I_max = self.I_max
                bw_min = self.bw_min  # Minimum physical width of the cluster in meters
                bw_max = self.bw_max  # Maximum physical width of the cluster in meters

                # Define the clustering zone around i_wrapped
                cluster_radius = self.cluster_radius  # Number of indexes to include on each side
                num_indexes = len(closest_scan.ranges)  # Total number of LiDAR indexes

                # Compute the range of indexes to consider, handling wrap-around
                start_index = (i_wrapped - cluster_radius) % num_indexes
                end_index = (i_wrapped + cluster_radius) % num_indexes

                if start_index < end_index:
                    zone_mask = np.zeros(num_indexes, dtype=bool)
                    zone_mask[start_index:end_index + 1] = True
                else:
                    # Wrap-around case: split into two ranges
                    zone_mask = np.zeros(num_indexes, dtype=bool)
                    zone_mask[start_index:] = True
                    zone_mask[:end_index + 1] = True

                # Apply the zone mask to ranges and intensities
                ranges_arr = np.array(closest_scan.ranges)
                intens_arr = np.array(closest_scan.intensities)
                valid_range_mask = np.isfinite(ranges_arr) & (ranges_arr > 0.1) & (ranges_arr <= 2.0)
                valid_intensity_mask = (intens_arr > I_min) & (intens_arr < I_max)

                
                if self.use_sim_time:
                    candidate_mask = valid_range_mask & zone_mask
                else:
                    candidate_mask = valid_range_mask & valid_intensity_mask & zone_mask


                # Adjacent-similar mask within the clustering zone
                deltas = np.abs(ranges_arr[:-1] - ranges_arr[1:]) <= same_object_thr
                adjacent_both_valid = candidate_mask[:-1] & candidate_mask[1:]
                cluster_mask_1d = deltas & adjacent_both_valid

                # Label connected runs
                labeled, num_clust = label(cluster_mask_1d)
                clusters = []
                for cid in range(1, num_clust + 1):
                    pos = np.where(labeled == cid)[0]
                    start = pos[0]
                    end = pos[-1] + 1
                    beam_indices = np.arange(start, end + 1)
                    clusters.append(beam_indices)

                # Filter clusters with more than 1 reading
                valid_clusters = [c for c in clusters if len(c) > 1]

                # Filter clusters based on physical width
                filtered_clusters = []
                for c in valid_clusters:
                    start_idx = c[0]  # First index of the cluster
                    end_idx = c[-1]  # Last index of the cluster

                    # Calculate the angular span of the cluster
                    angular_span = (end_idx - start_idx) * closest_scan.angle_increment  # Angular span in radians

                    # Use the median distance of the cluster to calculate the physical width
                    med_r = np.median(ranges_arr[c])  # Median distance of the cluster
                    cluster_width = 2 * med_r * math.sin(angular_span / 2)  # Physical width using trigonometry

                    if bw_min <= cluster_width <= bw_max:
                        filtered_clusters.append(c)
                        self.get_logger().info(
                            f"Cluster at indices {start_idx}-{end_idx} with width {cluster_width:.2f} m "
                            f"accepted (within range [{bw_min}, {bw_max}])."
                        )
                    else:
                        self.get_logger().info(
                            f"Cluster at indices {start_idx}-{end_idx} with width {cluster_width:.2f} m "
                            f"filtered out (outside range [{bw_min}, {bw_max}])."
                        )

                valid_clusters = filtered_clusters

                if not valid_clusters:
                    self.get_logger().warn("No valid clusters found. Exiting object detection process.")
                    return  # Exit the method early

                # Find the cluster closest to the detected red object index and nearest to the robot
                best_cluster = None
                best_score = float('inf')  # Lower score is better

                for c in valid_clusters:
                    center_idx = int(round(np.mean(c)))  # Center index of the cluster
                    distance = ranges_arr[center_idx]  # Distance of the cluster center

                    # Adjusted scoring formula:
                    # - Penalize clusters farther from the robot
                    # - Penalize clusters farther from the detected red object
                    proximity_penalty = abs(center_idx - i_wrapped)  # Penalize based on distance from i_wrapped
                    score = distance + 0.5 * proximity_penalty  # Distance has more weight than proximity penalty

                    self.get_logger().info(
                        f"Cluster center index: {center_idx}, Distance: {distance:.2f}, "
                        f"Proximity penalty: {proximity_penalty:.2f}, Score: {score:.2f}"
                    )

                    if score < best_score:
                        best_score = score
                        best_cluster = c

                # Use the best cluster
                if best_cluster is not None:
                    i_object = int(round(np.mean(best_cluster)))  # Center index of the best cluster
                    distance = ranges_arr[i_object]

                    # Final angle
                    final_angle = closest_scan.angle_min + i_object * closest_scan.angle_increment
                else:
                    self.get_logger().warn("No suitable cluster found. Exiting object detection process.")
                    return

                # Debug log for the final angle
                self.get_logger().info(f"Final angle for object: {final_angle:.2f} rad --- {final_angle * (180.0 / math.pi):.2f}°")
                self.get_logger().info(f"Final index: {i_object}")
                self.get_logger().info(f"Distance final index {i_object}: {distance:.2f}")

                # Call marker_creation
                self.marker_creation(final_angle, t_detect, distance)

        except (IndexError, ValueError) as e:
            self.get_logger().error(f'Error parsing message: {msg.data}. Exception: {e}')

    def scan_callback(self, scan_msg: LaserScan):
        """
        Buffer the most recent 20 LaserScan messages.
        """
        self.scan_buffer.append(scan_msg)
        #self.get_logger().debug(f"Buffered LiDAR scan at {scan_msg.header.stamp.sec}.{scan_msg.header.stamp.nanosec}, total buffered: {len(self.scan_buffer)}")
        #self.get_logger().info(f"Buffered LiDAR scan with {len(scan_msg.ranges)} ranges")

def main(args=None):
    rclpy.init(args=args)
    node = ExploreController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

