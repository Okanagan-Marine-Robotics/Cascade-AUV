import object_detection.detector 
from object_detection.detector import ObjectDetectorNode 
import rclpy
import numpy as np
import time
import cv2
import numpy as np

class ExampleDetector(ObjectDetectorNode):
    def __init__(self):
        super().__init__(node_name="example_detector")
        #define model here if needed

    def inference(self,rgb,depth):
        tolerance = 10

        # Convert the BGR color space of the image to HSV
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

        # Define the HSV range for detecting blue
        # Lower and Upper ranges for red color in HSV
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])


        # Create a binary mask where blue regions are set to 1 and others to 0
        # Create two masks for the two red ranges
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        

        # Combine the two masks & Normalize to 0 and 1
        binary_mask = cv2.bitwise_or(mask1, mask2) // 255

        # Calculate confidence based on distance to the center of each red range
        hue_channel = hsv[:, :, 0].astype(np.float32)  # Get hue channel in float32 for calculations
        confidence1 = np.clip((tolerance - np.abs(hue_channel - 5)) / tolerance, 0, 1) * 255
        confidence2 = np.clip((tolerance - np.abs(hue_channel - 175)) / tolerance, 0, 1) * 255


        # Combine the two confidence maps, taking the maximum confidence value at each pixel
        confidence = np.maximum(confidence1, confidence2).astype(np.uint8)

        # Optional: Display results for testing
        cv2.imshow('Binary Mask', binary_mask * 255)  # Display as 0 and 255
        cv2.imshow('Confidence Map', confidence)  # Confidence as a grayscale image
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # Return the binary mask for further processing
        # This is the old version of return statement, needs to be rewrite
        return binary_mask.astype(np.float32)

def main(args=None):
    rclpy.init(args=args)
    node = ExampleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

