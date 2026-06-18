#!/usr/bin/env python3
"""
Trash detection + semantic mapping + logging
ROS2 Humble / Ubuntu 22.04
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray

from cv_bridge import CvBridge
import cv2

from ultralytics import YOLO
from geometry_msgs.msg import PointStamped

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import sqlite3
import datetime
import os


TRASH_CLASSES = {
    'bottle', 'cup', 'can', 'wine glass',
    'bowl', 'book'
}

# Lowered from 0.40 — Gazebo's simplified, flat-shaded models (bowls, cups,
# tilted bottles) often score well below 0.40 against the real-photo-trained
# COCO model, so the old threshold was silently discarding valid detections.
CONFIDENCE_THRESHOLD = 0.20

DETECTION_FRAMES_REQUIRED = 2
LINEAR_SLOW_SPEED = 0.05  # m/s — creep speed on first detection frame before full stop
MIN_DISTANCE_BETWEEN_MARKERS = 0.5  # meters, avoid duplicate markers for same trash
MISS_FRAMES_ALLOWED = 5    # consecutive frames with no detection before resetting state
ALERT_COOLDOWN_SEC = 5.0   # minimum seconds between stop-alerts sent to patrol node

# Color per trash category (RGB, 0-1 range)
CLASS_COLORS = {
    'bottle': (0.0, 0.6, 1.0),
    'can': (1.0, 0.6, 0.0),
    'cup': (1.0, 0.6, 0.0),
    'bowl': (1.0, 0.0, 0.6),
    'wine glass': (0.6, 0.0, 1.0),
    'book': (0.0, 1.0, 0.4),
    'default': (1.0, 0.0, 0.0),
}

DB_PATH = os.path.expanduser('~/turtlebot3_ws/trash_log.db')


class TrashDetector(Node):

    def __init__(self):
        super().__init__('trash_detector')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.get_logger().info('Loading YOLOv8 model...')
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLOv8 model loaded.')

        self.bridge = CvBridge()
        self.detection_counter = 0
        self.miss_counter = 0
        self.robot_stopped = False
        self.alert_cooldown_active = False
        self._alert_cooldown_timer = None
        self.marker_id_counter = 0
        self.known_locations = []  # list of (x, y, trash_type) already marked

        # TF setup for getting robot's map-frame position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera subscriber
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/trash_markers', 10)
        self.alert_pub = self.create_publisher(PointStamped, '/trash_alert', 10)

        # SQLite setup
        self.setup_database()

        cv2.namedWindow('Eco CleanBot — Camera Feed', cv2.WINDOW_NORMAL)
        self.get_logger().info('Trash detector ready (with mapping + logging).')

    def setup_database(self):
        self.db_conn = sqlite3.connect(DB_PATH)
        cursor = self.db_conn.cursor()
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS detections (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT,
                trash_type TEXT,
                x REAL,
                y REAL,
                confidence REAL
            )
        ''')
        self.db_conn.commit()
        self.get_logger().info(f'Database ready at {DB_PATH}')

    def get_robot_pose(self):
        """Returns (x, y) of robot in map frame, or None if TF unavailable."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            return (x, y)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    def is_new_location(self, x, y, trash_type):
        """Check if this (location, type) pair has already been marked."""
        for (kx, ky, ktype) in self.known_locations:
            dist = ((x - kx) ** 2 + (y - ky) ** 2) ** 0.5
            if dist < MIN_DISTANCE_BETWEEN_MARKERS and ktype == trash_type:
                return False
        return True

    def publish_marker(self, x, y, trash_type):
        color = CLASS_COLORS.get(trash_type, CLASS_COLORS['default'])

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'trash'
        marker.id = self.marker_id_counter
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.2
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.9
        marker.lifetime = Duration(seconds=0).to_msg()  # 0 = forever

        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

        self.marker_id_counter += 1

    def log_detection(self, trash_type, x, y, confidence):
        timestamp = datetime.datetime.now().isoformat()
        cursor = self.db_conn.cursor()
        cursor.execute(
            'INSERT INTO detections (timestamp, trash_type, x, y, confidence) VALUES (?, ?, ?, ?, ?)',
            (timestamp, trash_type, x, y, confidence)
        )
        self.db_conn.commit()
        self.get_logger().info(f'Logged: {trash_type} at ({x:.2f}, {y:.2f})')

    def stop_robot(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def slow_robot(self):
        msg = Twist()
        msg.linear.x = LINEAR_SLOW_SPEED
        self.cmd_vel_pub.publish(msg)

    def _reset_alert_cooldown(self):
        self.alert_cooldown_active = False
        if self._alert_cooldown_timer is not None:
            self._alert_cooldown_timer.cancel()
            self._alert_cooldown_timer = None

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        results = self.model.predict(source=frame, verbose=False, conf=CONFIDENCE_THRESHOLD)

        detections = []
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0].item())
                class_name = self.model.names[class_id]
                confidence = float(box.conf[0].item())
                coords = box.xyxy[0].tolist()

                if class_name in TRASH_CLASSES:
                    detections.append({'name': class_name, 'conf': confidence, 'box': coords})
                    x1, y1, x2, y2 = map(int, coords)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f'{class_name} {confidence:.2f}', (x1, max(y1 - 10, 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if detections:
            self.miss_counter = 0
            self.detection_counter += 1

            if self.detection_counter >= DETECTION_FRAMES_REQUIRED:
                self.stop_robot()
                self.robot_stopped = True

                pose = self.get_robot_pose()
                if pose is not None:
                    x, y = pose

                    # Log/mark every detected item whose (location, type) pair
                    # is genuinely new. is_new_location is the ONLY gatekeeper
                    # here, so the same object never gets logged twice even
                    # while the robot sits still looking at it for many frames.
                    any_new_this_frame = False
                    for d in detections:
                        if self.is_new_location(x, y, d['name']):
                            self.publish_marker(x, y, d['name'])
                            self.log_detection(d['name'], x, y, d['conf'])
                            self.known_locations.append((x, y, d['name']))
                            any_new_this_frame = True
                            self.get_logger().info(
                                f"[TRASH DETECTED] {d['name']} conf={d['conf']:.2f} at ({x:.2f},{y:.2f})"
                            )
                        # else: already logged at this spot — skip silently,
                        # no need to spam [DUPLICATE] every single frame

                    # Only alert the patrol node (pause + avoid) when something
                    # NEW was actually logged this frame, not on every repeat
                    if any_new_this_frame and not self.alert_cooldown_active:
                        self.alert_cooldown_active = True
                        if self._alert_cooldown_timer is not None:
                            self._alert_cooldown_timer.cancel()
                        self._alert_cooldown_timer = self.create_timer(
                            ALERT_COOLDOWN_SEC, self._reset_alert_cooldown
                        )
                        alert = PointStamped()
                        alert.header.frame_id = 'map'
                        alert.header.stamp = self.get_clock().now().to_msg()
                        alert.point.x = x
                        alert.point.y = y
                        self.alert_pub.publish(alert)
                else:
                    self.get_logger().warn('Could not get robot pose, skipping marker/log')

                cv2.putText(frame, 'TRASH DETECTED — STOPPED', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                self.slow_robot()
                cv2.putText(frame, 'TRASH DETECTED — SLOWING', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
        else:
            # Tolerate brief detection gaps (close-range camera blind spot, lighting flicker)
            # before resetting — prevents premature counter reset on a single missed frame
            self.miss_counter += 1
            if self.miss_counter > MISS_FRAMES_ALLOWED:
                self.detection_counter = 0
                self.robot_stopped = False
                self.miss_counter = 0

        cv2.imshow('Eco CleanBot — Camera Feed', frame)
        key = cv2.waitKey(10)


def main(args=None):
    rclpy.init(args=args)
    node = TrashDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    finally:
        node.db_conn.close()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
