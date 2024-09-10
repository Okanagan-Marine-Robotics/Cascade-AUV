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
        self.video_writer_color = None
        self.video_writer_depth = None

    def synced_callback(self, image_msg ,depth_msg):   
        color_frame = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        # Convert depth frame to a format suitable for video saving
        depth_frame_normalized = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX)
        depth_frame_normalized = cv2.convertScaleAbs(depth_frame_normalized)

        depth_frame_colored = cv2.applyColorMap(depth_frame_normalized, cv2.COLORMAP_JET)

        if self.video_writer_color is None:
            height, width, _ = color_frame.shape
            self.video_writer_color = cv2.VideoWriter('color_output.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 14.0, (width, height))
        if self.video_writer_depth is None:
            height, width, _ = depth_frame_colored.shape
            self.video_writer_depth = cv2.VideoWriter('depth_output.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 14.0, (width, height))

        # Write the combined frame to the video file
        self.video_writer_color.write(color_frame)
        self.video_writer_depth.write(depth_frame_colored)

    def destroy(self):
        if self.video_writer_color:
            self.video_writer_color.release()
        if self.video_writer_depth:
            self.video_writer_depth.release()


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()

if __name__ == '__main__':
    main()

