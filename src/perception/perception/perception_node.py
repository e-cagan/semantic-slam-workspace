"""
Module for perception node.
"""

import numpy as np
import cv2
import math

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from ultralytics import YOLO

from sensor_msgs.msg import Image, LaserScan, CameraInfo
from semantic_interfaces.msg import SemanticObservation


# Helper Functions
def is_valid(idx):
    """
    A helper function that filters out 0.0 and inf values on laser scan which are invalid values.
    """

    if idx != 0.0 and idx != math.inf:
        return idx


# Perception node
class PerceptionNode(Node):
    """
    A node that detects objects using YOLO.
    """

    def __init__(self):
        super().__init__('perception_node')

        # Parameters
        self.declare_parameter("yolo_model_path", "yolov8n.pt")
        self.declare_parameter("image_size", 640)
        self.declare_parameter("conf_threshold", 0.35)

        # Publishers
        self.pub = self.create_publisher(SemanticObservation, "/semantic/observations", 10)

        # Subscribers
        self.img_sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, "/camera/camera_info", self.cam_info_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)

        # Others
        self.cv_bridge = CvBridge()
        self.model_path = self.get_parameter("yolo_model_path").value
        self.model = YOLO(self.model_path)
        self.imgsz = self.get_parameter("image_size").value
        self.conf = self.get_parameter("conf_threshold").value
        self.cam_matrix = None
        self.dist_coeffs = None
        self.last_scan = None


    # Camera info callback
    def cam_info_callback(self, msg):
        """
        A callback that saves camera matrix and distortion coefficients.
        """

        # Convert cam matrix to 3x3 array
        self.cam_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    
    # Scan callback
    def scan_callback(self, msg):
        """
        A callback that assigns the last scan based on scan message value.
        """

        self.last_scan = msg

    
    # Image callback
    def image_callback(self, msg):
        """
        A callback for object detection.
        """

        # Convert ros image to numpy array
        img = self.cv_bridge.imgmsg_to_cv2(img_msg=msg, desired_encoding="bgr8")

        # Convert image to RGB since YOLO expects RGB
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Predict the detections using YOLO
        results = self.model.predict(
            source=img,
            imgsz=self.imgsz,
            conf=self.conf,
        )

        # Iterate trough bboxes to display predictions
        for box in results[0].boxes:
            print(f"Label: {self.model.names[int(box.cls)]}  Confidence: {box.conf}  Bbox Center X: {box.xywh[0][0].item()}")

            # LIDAR fusion
            # Check camera matrix and last scan exists
            if self.cam_matrix is None:
                return
            if self.dist_coeffs is None:
                return
            if self.last_scan is None:
                return
            
            # Take fx and cx
            fx = self.cam_matrix[0][0]
            cx = self.cam_matrix[0][2]

            # Calculate u (a.k.a center x) for camera angle
            u = box.xywh[0][0].item()

            # Convert pixels to angles
            angle = -np.arctan2(u - cx, fx)
            angular_width = np.arctan2(box.xywh[0][2].item() / 2, fx)

            # Turn negative angles to positive
            if angle < 0:
                angle += 2 * math.pi

            # Calculate indexes
            index = (angle - self.last_scan.angle_min) / self.last_scan.angle_increment
            if index < 0:
                index = len(self.last_scan.ranges) + index

            # Interval slicing for angle and index
            min_angle = angle - angular_width
            max_angle = angle + angular_width

            # Prune angle values between 0 and 2 * pi
            min_angle = min_angle % (2 * math.pi)
            max_angle = max_angle % (2 * math.pi)

            min_index = int(min_angle / self.last_scan.angle_increment)
            max_index = int(max_angle / self.last_scan.angle_increment)

            # Turn negative indexes to positive
            if min_index < 0:
                min_index = len(self.last_scan.ranges) + min_index
            if max_index < 0:
                max_index = len(self.last_scan.ranges) + max_index

            # Take ranged indexes and filter them. Lastly take the minimum of it
            if min_index > max_index:
                # wrap-around situation
                distances = self.last_scan.ranges[min_index:] + self.last_scan.ranges[:max_index]
            else:
                distances = self.last_scan.ranges[min_index:max_index]
            distances = list(filter(is_valid, distances))
            distance = min(distances) if distances else None

            print(f"Distance: {distance}")


def main(args=None):
    """
    Main function that handles node lifecycle.
    """

    # Node lifecycle
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()