import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import os
import keras
from keras import layers
from tensorflow import data as tf_data
import matplotlib.pyplot as plt
import tensorflow as tf
import logging
from tensorflow.python.client import device_lib
from keras.models import load_model
from message_filters import ApproximateTimeSynchronizer, Subscriber

class ObjectDetectorNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.bridge = CvBridge()
        queue_size = 2
        acceptable_delay = 0.06  # seconds
        tss = ApproximateTimeSynchronizer(
            [Subscriber(self, Image, "/camera/rgb"),
             Subscriber(self, Image, "/camera/depth"),
             ],
            queue_size,
            acceptable_delay)
        self.target_size = (480, 640, 2)  # Example size (you can adjust this to your needs)
        tss.registerCallback(self.synced_callback)
        self.publisher_ = self.create_publisher(Image, '/labeled_image', 10)

    def inference(self,rgb,depth):
        #rgb and depth are both opencv images / numpy arrays
        #rgb is actually in bgr8 format, and depth is in 16UC1
        #each pixel of the depth image represents the distance in meters from the camera * 1000
        #so to get meters multipy the value by 0.001
        #if the value needs to be normalized, id say convert it to a scale from 0-1.0 where 1.0 is 8 meters away
        #anything above 8 meters away starts to become less accurate
        return np.zeros(self.target_size, dtype=np.float32)
        #inference method should return a numpy array

    def synced_callback(self, rgb_msg, depth_msg):
        color_frame = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        label_image = self.inference(color_frame, depth_frame)#image must be returned in 32FC2 format, [class, confidence]
        
        try:
            # Convert the empty image to ROS Image message
            label_msg = self.bridge.cv2_to_imgmsg(label_image, encoding='32FC2')
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert empty image: {e}")
            return

        label_msg.header.stamp = rgb_msg.header.stamp
        self.publisher_.publish(label_msg)
