import object_detection.detector 
from object_detection.detector import ObjectDetectorNode 
import rclpy
import numpy as np
import os

os.environ["KERAS_BACKEND"] = "jax"  # @param ["tensorflow", "jax", "torch"]

from tensorflow import data as tf_data
import tensorflow_datasets as tfds
import keras
import keras_cv
from keras_cv import bounding_box
from keras_cv import visualization
import tqdm
import matplotlib.pyplot as plt
import cv2

class ExampleDetector(ObjectDetectorNode):
    def __init__(self):
        super().__init__(node_name="yolo_detector")
        prediction_decoder = keras_cv.layers.NonMaxSuppression(
            bounding_box_format="xyxy",
            from_logits=True,
            # Decrease the required threshold to make predictions get pruned out
            iou_threshold=0.2,
            # Tune confidence threshold for predictions to pass NMS
            confidence_threshold=0.7,
        )
        self.pretrained_model = keras_cv.models.YOLOV8Detector.from_preset(
            "yolo_v8_m_pascalvoc",
            bounding_box_format="xyxy",
            prediction_decoder=prediction_decoder,
        )
        self.inference_resizing = keras_cv.layers.Resizing(
            640, 640, pad_to_aspect_ratio=True, bounding_box_format="xyxy"
        )

    def inference(self,rgb,depth):
        #initializing pretrained model
        label_img = np.zeros(self.target_size, dtype=np.float32)

        image_batch = self.inference_resizing([rgb])#no need to convert rgb, its already in numpy array format
        
        #running the inference
        #pred = self.pretrained_model(image_batch, training=False)
        pred = self.pretrained_model.predict(image_batch)
        '''
        visualization.plot_bounding_box_gallery(
            image_batch,
            value_range=(0, 255),
            rows=1,
            cols=1,
            y_pred=pred,
            scale=5,
            font_scale=0.7,
            bounding_box_format="xyxy",
        )
        plt.show()
        self.get_logger().info('"%s"' % pred['boxes'][0][0])
        '''
        xyxy = pred['boxes'][0][0]
        cv2.rectangle(label_img, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), 1.0, -1)# for each bounding box, not worrying about overlapping boxes for now
        

        cv2.imshow("label",label_img[:,:, 0])
        cv2.waitKey(1)
        return label_img

def main(args=None):
    rclpy.init(args=args)
    node = ExampleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

