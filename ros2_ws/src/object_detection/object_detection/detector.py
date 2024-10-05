from image_proc import image_node
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

#this is currently dummy code that publishes nothing

class EmptyImageNode(image_node.ImageNode):
    def __init__(self):
        super().__init__(sub_topic="camera/rgb", pub_topic="/labeled_image", name="object_detector")
        self.bridge = CvBridge()

    def subscription_callback(self, msg):
        # Instead of processing the received image, we'll create a blank F32C2 image of size 640x480

        # Set dimensions of the empty image
        height, width = 480, 640

        # Create an empty image (all zeros), with 2 channels (F32C2)
        empty_image = np.zeros((height, width, 2), dtype=np.float32)
        color_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow('D455 RGB Image', color_frame)
        cv2.waitKey(1)

        try:
            # Convert the empty image to ROS Image message
            empty_msg = self.bridge.cv2_to_imgmsg(empty_image, encoding='32FC2')
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert empty image: {e}")
            return

        # Copy the timestamp from the original message
        empty_msg.header.stamp = msg.header.stamp

        # Publish the empty image
        self.publisher_.publish(empty_msg)

def main(args=None):
    rclpy.init(args=args)
    empty_image_node = EmptyImageNode()
    rclpy.spin(empty_image_node)
    empty_image_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

