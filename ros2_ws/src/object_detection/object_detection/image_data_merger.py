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
        msg = ImageWithPose()
        msg.depth = depth_msg
        msg.rgb = rgb_msg
        msg.label = label_msg
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

#TODO DELETE THIS NODE

