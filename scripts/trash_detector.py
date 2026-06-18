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
