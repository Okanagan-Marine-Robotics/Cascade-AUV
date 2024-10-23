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
        

        #cv2.imshow("label",label_img[:,:, 0])
        '''
        #IMPORTANT NOTE:
        #the following 40 or so lines are very jank i am sorry
        #their purpose is to take the bounding box from label_image and filter out all pixels that arent in the foreground.
        #this can probably be done a bit better though
        class_image = label_img[:, :, 0]
        #confidence_image = label_image[:, :, 1]

        # Create a new image with an extra dimension to store class and confidence
        height, width = depth.shape
        combined_image = np.zeros((height, width, 2), dtype=np.float32)

        # Create a copy of class and confidence images to avoid modifying the originals
        labeled_class_image = np.zeros_like(class_image)
        #labeled_confidence_image = np.zeros_like(confidence_image)

        # Process each unique class in the class_image
        unique_classes = np.unique(class_image)
        for detected_class in unique_classes:
            if detected_class == 0:
                continue  # Assuming class 0 is the background or an invalid class

            # Create a mask for the current class
            class_mask = (class_image == detected_class)

            # Get the depth values within the mask
            depth_patch = depth[class_mask]  

            if depth_patch.size == 0:
                continue  # Skip if no pixels are found for this class

            # Find the minimum and maximum depth values within the mask
            min_depth = np.min(depth_patch)

            # Calculate the threshold depth value
            threshold_depth = min_depth * 1.5 
            # Find the pixels within the desired depth range
            depth_mask = class_mask
            #(depth > 0) & (depth <= threshold_depth)

            # Label the pixels within the depth range
            labeled_class_image[depth_mask] = detected_class
            #labeled_confidence_image[depth_mask] = 1.0  # Set confidence to 1.0

        combined_image[:, :, 0] = labeled_class_image
        #combined_image[:, :, 1] = labeled_confidence_image
        cv2.imshow("label",labeled_class_image)
        cv2.waitKey(1)
        '''
        return label_img
def main(args=None):
    rclpy.init(args=args)
    node = ExampleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

