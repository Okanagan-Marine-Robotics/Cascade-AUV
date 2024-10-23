import object_detection.detector 
from object_detection.detector import ObjectDetectorNode 
import rclpy
import numpy as np
import cv2

class ExampleDetector(ObjectDetectorNode):
    def __init__(self):
        super().__init__(node_name="dummy_detector", using_keras=False)

    def inference(self,rgb,depth):
        #insert inferencing code here
        label_img = np.zeros(self.target_size, dtype=np.float32)
        cv2.rectangle(label_img, (0,0),(300,300),1.0, -1)
        cv2.imshow("label",label_img[:,:,0])
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

