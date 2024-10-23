import object_detection.detector 
from object_detection.detector import ObjectDetectorNode 
import rclpy
import numpy as np
import time
class ExampleDetector(ObjectDetectorNode):
    def __init__(self):
        super().__init__(node_name="example_detector")
        #define model here if needed

    def inference(self,rgb,depth):
        #insert inferencing code here
        return np.zeros(self.target_size, dtype=np.float32)#returns nothing

def main(args=None):
    rclpy.init(args=args)
    node = ExampleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

