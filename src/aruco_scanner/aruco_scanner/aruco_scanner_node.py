#!/usr/bin/env python3
"""
aruco_scanner.node

Behavior:
1) On startup, reads the first /odometry/filtered message and stores it as HOME pose/orientation.
2) Enters SCANNING state: publishes angular z=0.5 on /cmd_vel to rotate and listens to /aruco_detections.
   - For each marker seen, requires continuous detection >= 0.2 s for that marker to be considered valid.
   - Collects unique valid marker IDs until 5 are found.
3) Switches to CENTERING state: choose the marker with the smallest ID and attempt to center so marker.pose.position.x ∈ [-0.03, 0.03].
   - Simple proportional-like controller using angular z commands.
4) DONE: stops motion.

This node attempts to import the detection message dynamically (so it doesn't hard-code package names).
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from aruco_opencv_msgs.msg import ArucoDetection
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoScanner(Node):
    STATE_INIT_HOME = "INIT_HOME"
    STATE_SCANNING = "SCANNING"
    STATE_CENTERING = "CENTERING"
    STATE_APPROACHING = "APPROACHING"
    STATE_HOME = "HOME"
    STATE_DONE = "DONE"

    def __init__(self):
        super().__init__("aruco_scanner_node")

        # Parameters (tweakable)
        self.declare_parameter("detection_topic", "/aruco_detections")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("spin_speed", 0.5)          # angular z used during scanning
        self.declare_parameter("center_speed", 0.5)       # angular z used during centering
        self.declare_parameter("min_detection_time", 0.7)  # seconds required to validate a marker
        self.declare_parameter("num_markers_required", 5)
        self.declare_parameter("center_tolerance", 0.03)   # ± tolerance on marker.pose.position.x
        self.declare_parameter("approaching_dist_tolerance", 0.05) 
        self.declare_parameter("approaching_speed", 0.1)


        # Load params
        self.detection_topic = self.get_parameter("detection_topic").get_parameter_value().string_value
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self.spin_speed = self.get_parameter("spin_speed").get_parameter_value().double_value
        self.center_speed = self.get_parameter("center_speed").get_parameter_value().double_value
        self.min_detection_time = self.get_parameter("min_detection_time").get_parameter_value().double_value
        self.num_markers_required = int(self.get_parameter("num_markers_required").get_parameter_value().integer_value)
        self.center_tol = self.get_parameter("center_tolerance").get_parameter_value().double_value
        self.approaching_dist_tol = self.get_parameter("approaching_dist_tolerance").get_parameter_value().double_value
        self.approaching_speed = self.get_parameter("approaching_speed").get_parameter_value().double_value
        
        self.bridge = CvBridge()
        self.fx = 381.3611602783203   # focal length x
        self.fy = 381.3611602783203   # focal length y
        self.cx = 320.0   # principal point x
        self.cy = 240.0   # principal point y
        self.marker_size = 0.0742
        # State
        self.state = self.STATE_INIT_HOME
        self.home_odom = None
        self.current_odom = None
        # Marker tracking structures:
        # markers_seen: id -> { first_seen_ns, last_seen_ns, confirmed(bool), last_pose }
        self.markers_seen = {}
        self.current_seen_markers = {}
        self.valid_marker_ids = []  # order by detection (unique)
        self.markers_completed = []

        self.target_marker_id = None
        self.image_sub = self.create_subscription(
            Image,
            "/camera/image",
            self.image_callback,
            10
        )

        self.annotated_pub = self.create_publisher(
            Image,
            "/camera/annotated",
            10
        )
        # Publisher for cmd_vel
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Subscribe to odom (to record home on first message)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )



        # Subscribe to detections
        self.det_sub = self.create_subscription(
            ArucoDetection, self.detection_topic, self.detections_callback, qos_profile_sensor_data
        )

        # Timer that runs the state machine at 10 Hz and republishes velocities
        self.timer = self.create_timer(0.1, self._timer_callback)

        self.get_logger().info("ArucoScanner node started. Waiting for /odometry/filtered to set HOME...")

    # --------------------------
    # Callbacks
    # --------------------------
    def odom_callback(self, msg: Odometry):
        # On first odom message, record home and move to SCANNING
        if self.home_odom is None:
            self.home_odom = msg
            self.get_logger().info("Home odometry recorded (from /odometry/filtered).")
            # Switch to scanning state
            self.state = self.STATE_SCANNING
        self.current_odom = msg

    def image_callback(self, msg):
        # Convert ROS -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Draw markers
        for marker in self.current_seen_markers:  
            # marker.pose is in CAMERA FRAME as per your setup
            position = self.current_seen_markers[marker]["last_pose"].position
            x = position.x
            y = position.y
            z = position.z
            pos = (
                x,
                y,
                z
            )

            cx, cy, radius = self.project_marker_circle(pos, self.marker_size)

            if radius is None:
                continue

            u, v = self.project_point_to_image(x, y, z)

            if u is None or v is None:
                continue

            # Only draw if inside image bounds
            if (0 <= cx < frame.shape[1]) and (0 <= cy < frame.shape[0]):
                cv2.circle(frame, (cx, cy), radius, (0, 255, 0), 2)
                cv2.putText(
                    frame,
                    f"ID {marker}",
                    (cx + radius + 5, cy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )


        # Publish annotated image
        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.annotated_pub.publish(out_msg)
    def project_marker_circle(self, pos, size):
        """
        Given marker center position (x,y,z) and side length (meters),
        compute the circle center and radius in pixel space.
        """
        x, y, z = pos

        # Marker half-size
        h = size / 2.0

        # Define marker corners in local marker coordinates
        corners = [
            (-h, -h, 0),
            ( h, -h, 0),
            ( h,  h, 0),
            (-h,  h, 0),
        ]

        # Project corners into image
        projected = []
        for cx, cy, cz in corners:
            # Marker is assumed to have no rotation relative to camera
            X = x + cx
            Y = y + cy
            Z = z + cz if z + cz != 0 else z

            u = int(self.fx * X / Z + self.cx)
            v = int(self.fy * Y / Z + self.cy)
            projected.append((u, v))

        # Compute center in pixel space
        center_u, center_v = self.project_point_to_image(x, y, z)

        # Compute radius: maximum distance corner → center
        radius = max(
            int(np.hypot(u - center_u, v - center_v))
            for u, v in projected
        )

        return (center_u, center_v, radius)
    def project_point_to_image(self, x, y, z):
        """
        Project 3D point in camera frame into the image plane.
        """
        if z <= 0:
            return None, None  # behind camera or invalid

        u = (self.fx * x) / z + self.cx
        v = (self.fy * y) / z + self.cy
        return int(u), int(v)
    def detections_callback(self, msg):
        """
        Expects a message that includes a 'markers' sequence, where each marker has:
        - marker_id (int)
        - pose.position.x, pose.position.y, pose.position.z
        - optionally pose.orientation
        The function is intentionally generic to match your provided sample schema.
        """
        now_ns = self.get_clock().now().nanoseconds

        # Extract markers list permissively
        markers = []
        if hasattr(msg, "markers"):
            markers = list(msg.markers)
        else:
            # last resort: try common names
            # log and return
            self.get_logger().warn("Detection message has no attribute 'markers'. Ignoring this message.")
            return

        seen_ids_this_msg = set()
        self.current_seen_markers = {}
        for m in markers:
            # try to extract id and pose in several likely field names
            marker_id = m.marker_id
            pose = m.pose
            self.current_seen_markers[marker_id] = {
                    "last_pose": pose,
                }
            # store last_pose and timestamps
            seen_ids_this_msg.add(marker_id)
            entry = self.markers_seen.get(marker_id)
            if entry is None:
                # new candidate
                self.markers_seen[marker_id] = {
                    "first_seen_ns": now_ns,
                    "last_seen_ns": now_ns,
                    "confirmed": False,
                    "last_pose": pose,
                }
            elif not entry["confirmed"]:
                # update last_seen and pose; if previously missing for some time, reset first_seen
                time_since_last_seen = (now_ns - entry["last_seen_ns"]) / 1e9
                if time_since_last_seen > 0.1:
                    # gap -> restart detection window
                    entry["first_seen_ns"] = now_ns
                entry["last_seen_ns"] = now_ns
                entry["last_pose"] = pose

        # For markers not in this message, we don't immediately remove them; we check age in timer callback.
        # After updating, check confirmation condition for any candidate
        for marker_id, entry in list(self.markers_seen.items()):
            if not entry["confirmed"]:
                # confirmed if continuous detection >= min_detection_time
                elapsed = (now_ns - entry["first_seen_ns"]) / 1e9
                if elapsed >= self.min_detection_time:
                    entry["confirmed"] = True
                    if marker_id not in self.valid_marker_ids:
                        self.valid_marker_ids.append(marker_id)
                        self.get_logger().info(f"Marker {marker_id} validated after {elapsed:.3f} s. Collected: {len(self.valid_marker_ids)}/{self.num_markers_required}")

    # --------------------------
    # Timer / state machine
    # --------------------------
    def _timer_callback(self):
        now = self.get_clock().now().nanoseconds
        # Purge / reset markers that have not been seen recently ( > 0.2s )
        timeout_s = 0.2
        for marker_id, entry in list(self.markers_seen.items()):
            age_last_seen = (now - entry["last_seen_ns"]) / 1e9
            if age_last_seen > timeout_s:
                # If it was confirmed, keep it in valid_marker_ids (we don't remove confirmed ones),
                # but reset its entry so if it appears again it must be rediscovered.
                if not entry["confirmed"]:
                    # remove unconfirmed candidate
                    del self.markers_seen[marker_id]
                else:
                    # keep confirmed but allow last_pose to become stale
                    pass

        # State machine
        if self.state == self.STATE_INIT_HOME:
            # Waiting for odometry to set home; do nothing but publish zero velocity
            self._publish_twist(0.0, 0.0)
            return

        if self.state == self.STATE_SCANNING:
            # If not yet collected enough valid markers, keep rotating at spin_speed
            if len(self.valid_marker_ids) < self.num_markers_required:
                self._publish_twist(0.0, self.spin_speed)
            else:
                self.get_logger().info(f"Collected {len(self.valid_marker_ids)} valid markers. Switching to CENTERING.")
                # pick target marker: smallest id
                self.target_marker_id = min(self.valid_marker_ids)
                self.state = self.STATE_CENTERING
                self._publish_twist(0.0, 0.0)  # brief stop before centering
            return

        if self.state == self.STATE_CENTERING:
            # Ensure we have the latest pose for the target marker
            if self.target_marker_id not in self.current_seen_markers:
                # If marker not recently seen but is in confirmed list, we still may have last_pose
                self.get_logger().warn(f"Target marker {self.target_marker_id} has no fresh detection. Rotating slowly to reacquire.")
                self._publish_twist(0.0, self.center_speed)
                return

            entry = self.current_seen_markers[self.target_marker_id]

            # use pose.position.x for centering as requested
            marker_x = entry["last_pose"].position.x
            self.get_logger().debug(f"Centering: marker {self.target_marker_id} x={marker_x:.4f}")

            if -self.center_tol <= marker_x and marker_x <= self.center_tol:
                self.get_logger().info(f"Marker {self.target_marker_id} centered (x={marker_x:.4f}). Approaching.")
                self._publish_twist(0.0, 0.0)
                self.state = self.STATE_APPROACHING
                return

            # otherwise choose rotation direction:
            # NOTE: coordinate frames differ between systems. This controller assumes:
            # - if marker_x < -tol -> rotate left (positive angular z)
            # - if marker_x > +tol -> rotate right (negative angular z)
            # You may invert signs if your camera frame uses a different handedness.
            if marker_x < -self.center_tol:
                angz = +self.center_speed
            elif marker_x > self.center_tol:
                angz = -self.center_speed
            else:
                angz = 0.0

            self._publish_twist(0.0, angz)
            return
        if self.state == self.STATE_APPROACHING:
            # if marker is out of FOV, then we reached it
            if self.target_marker_id not in self.current_seen_markers:
                self.get_logger().info(f"Approached: marker {self.target_marker_id} out of FOV")
                self.state = self.STATE_HOME
                return
            # if we are within a certain distance from the marker we reached it
            entry = self.current_seen_markers[self.target_marker_id]
            marker_z = entry["last_pose"].position.z
            self.get_logger().debug(f"Approaching: marker {self.target_marker_id} z={marker_z:.4f}")
            if marker_z < self.approaching_dist_tol:
                self.get_logger().info(f"Approached: marker {self.target_marker_id} z={marker_z:.4f}")
                self.state = self.STATE_HOME
                return
            # Otherwise, reach the marker
            self._publish_twist(self.approaching_speed, 0.0)
        if self.state == self.STATE_HOME:
            # if reached home and all markers visited -> DONE
            if self._is_home() and len(self.markers_completed) == len(self.valid_marker_ids):
                self.get_logger().info("All markers visited. DONE.")
                self.state = self.STATE_DONE
                return
            
            if self._is_home():
                if self.target_marker_id not in self.markers_completed:
                    self.markers_completed.append(self.target_marker_id)
                # otherwise pick the next unvisited marker
                remaining = [m for m in self.valid_marker_ids if m not in self.markers_completed]
                if len(remaining) == 0:
                    self.get_logger().info("All markers visited. DONE.")
                    self.state = self.STATE_DONE
                    return
                self.target_marker_id = min(remaining)
                self.get_logger().info(f"Returning home OK. Next marker: {self.target_marker_id}. Switching to CENTERING.")
                self.state = self.STATE_CENTERING
                return
            self._publish_twist(-self.approaching_speed, 0.0)
        if self.state == self.STATE_DONE:
            # stop publishing motion
            self._publish_twist(0.0, 0.0)
            # remain in DONE; user can decide to shut down or reset externally
            return

    def _publish_twist(self, linear_x: float, angular_z: float):
        t = Twist()
        t.linear.x = linear_x
        t.linear.y = 0.0
        t.linear.z = 0.0
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = angular_z
        self.cmd_pub.publish(t)
    def _is_home(self):
        if self.home_odom is None or self.current_odom is None:
            return False
        dx = self.current_odom.pose.pose.position.x - self.home_odom.pose.pose.position.x
        dy = self.current_odom.pose.pose.position.y - self.home_odom.pose.pose.position.y
        dist = (dx*dx + dy*dy)**0.5
        return dist < 0.10   # 10 cm tolerance (adjust)
    def destroy_node(self):
        # ensure stop commanded before shutdown
        self._publish_twist(0.0, 0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArucoScanner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
