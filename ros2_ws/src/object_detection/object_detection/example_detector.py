import object_detection.detector 
from object_detection.detector import ObjectDetectorNode 
import rclpy
import numpy as np

class ExampleDetector(ObjectDetectorNode):
    def __init__(self):
        super().__init__(node_name="example_detector", model_file="model.keras")

    def inference(self,rgb,depth):
        return np.zeros(self.target_size, dtype=np.float32)

def main(args=None):
    rclpy.init(args=args)
    node = ExampleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

