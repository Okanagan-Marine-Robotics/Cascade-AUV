import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cascade_msgs.msg import ImageWithPose
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import cv2

class DepthLabelerNode(Node):
    def __init__(self):
        super().__init__("depth_labeler")
        self.bridge = CvBridge()
        queue_size = 20
        acceptable_delay = 0.06  # seconds
        tss = ApproximateTimeSynchronizer(
            [Subscriber(self, Image, "/depth_map"),
             Subscriber(self, Image, "/camera/rgb"),
             Subscriber(self, Image, "/labeled_image"),
             Subscriber(self, PoseStamped, "/pose")],
            queue_size,
            acceptable_delay)
        tss.registerCallback(self.synced_callback)
        self.publisher_ = self.create_publisher(ImageWithPose, '/semantic_depth_with_pose', 10)

    def synced_callback(self, depth_msg, rgb_msg ,label_msg, pose_msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            label_image = self.bridge.imgmsg_to_cv2(label_msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert images: {e}")
            return

        # Assuming label_image has two channels: class and confidence
        class_image = label_image[:, :, 0]
        confidence_image = label_image[:, :, 1]

        # Create a new image with an extra dimension to store class and confidence
        height, width = depth_image.shape
        combined_image = np.zeros((height, width, 2), dtype=np.float32)
        combined_image[:, :, 0] = confidence_image
        combined_image[:, :, 1] = class_image
        
        cv2.imshow('RGB Image', rgb_image)
        cv2.imshow('Class Image', class_image)
        cv2.imshow('Depth Image', depth_image)
        cv2.waitKey(1)

        # Convert the combined image back to ROS Image message
        try:
            combined_msg = self.bridge.cv2_to_imgmsg(combined_image, encoding="32FC2")
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="bgr8")
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert combined image: {e}")
            return

        msg = ImageWithPose()
        msg.depth = depth_msg
        msg.rgb = rgb_msg
        msg.label = combined_msg
        msg.pose = pose_msg.pose
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DepthLabelerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

