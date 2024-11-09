import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cascade_msgs.msg import ImageWithPose
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import cv2

class DataCollectorNode(Node):
    def __init__(self):
        super().__init__("data_collector")
        self.bridge = CvBridge()
        queue_size = 20
        acceptable_delay = 0.06  # seconds
        tss = ApproximateTimeSynchronizer(
            [Subscriber(self, Image, "/camera/camera/color/image_raw"),
             Subscriber(self, Image, "/camera/camera/depth/image_rect_raw"),
             ],
            queue_size,
            acceptable_delay)
        tss.registerCallback(self.synced_callback)
        self.file_num = 0
        self.prefix = "/home/ubuntu/data/tims_cup_0"

    def synced_callback(self, image_msg ,depth_msg):   
        color_frame = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
        
        # Write the combined frame to photo file

        self.get_logger().info(f'writing to {self.prefix}/{self.file_num}_png')

        cv2.imwrite(f"{self.prefix}/{self.file_num}_color.png", color_frame)
        cv2.imwrite(f"{self.prefix}/{self.file_num}_depth.png", depth_frame)
        self.file_num += 1

def main(args=None):
    rclpy.init(args=args)
    node = DataCollectorNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

