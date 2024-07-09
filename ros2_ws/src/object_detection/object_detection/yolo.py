from image_proc import image_node
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import torch
from pathlib import Path
from ultralytics import YOLO


class YoloNode(image_node.ImageNode):
    def __init__(self):
        super().__init__(sub_topic="/sensors/camera/rgb", pub_topic="/labeled_image", name="yolo")
        self.bridge = CvBridge()
        self.model = YOLO('best.pt')
        
        # Initialize YOLO model
        
    def inference(self, image):
        # Perform inference using the YOLO model
        results = self.model(image)

        # Process the results to extract bounding boxes
        boundingBoxes = []
        for result in results:
            for box in result.boxes:
                # Extract xywh from box correctly
                xywh = box.xywh[0]

                boundingBox = {
                'class_id': int(box.cls)+1,  # Class ID
                'confidence': float(box.conf),  # Confidence score
                'x': int(xywh[0] - xywh[2] / 2),  # X coordinate
                'y': int(xywh[1] - xywh[3] / 2),  # Y coordinate
                'w': int(xywh[2]),  # Width
                'h': int(xywh[3])  # Height
                }
                boundingBoxes.append(boundingBox)
        return boundingBoxes

    def subscription_callback(self, msg):
        original_stamp=self.get_clock().now().to_msg()
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Run inference
        boundingBoxes = self.inference(cv_image)

        # Create labeled image
        height, width, _ = cv_image.shape
        labeled_image = np.zeros((height, width, 2), dtype=np.float32)
        
        for box in boundingBoxes:
            class_id = box['class_id']
            confidence = box['confidence']
            x, y, w, h = box['x'], box['y'], box['w'], box['h']
            x = max(0, min(x, width - 1))
            y = max(0, min(y, height - 1))
            w = max(0, min(w, width - x))
            h = max(0, min(h, height - y))
            
            # Create slices for the region of interest
            roi_x = slice(x, x + w)
            roi_y = slice(y, y + h)

            # Update class and confidence images using NumPy's vectorized operations
            mask = confidence > labeled_image[roi_y, roi_x, 1]
            labeled_image[roi_y, roi_x, 0] = np.where(mask, class_id, labeled_image[roi_y, roi_x, 0])
            labeled_image[roi_y, roi_x, 1] = np.where(mask, confidence, labeled_image[roi_y, roi_x, 1])

        try:
            # Convert labeled image to ROS Image message
            labeled_msg = self.bridge.cv2_to_imgmsg(labeled_image, encoding="32FC2")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert labeled image: {e}")
            return

        # Publish the labeled image
        labeled_msg.header.stamp = original_stamp
        self.publisher_.publish(labeled_msg)

def main(args=None):
    rclpy.init(args=args)
    yolo_node = YoloNode()
    rclpy.spin(yolo_node)
    yolo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
