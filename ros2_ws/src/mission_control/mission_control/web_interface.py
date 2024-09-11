import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from flask import Flask, Response
import threading
from dash import Dash, html, dcc

# ROS 2 subscriber node for image
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.image_subscription = self.create_subscription(
            Image,  
            '/camera/camera/color/image_raw',  # Replace with your image topic
            self.image_callback,
            10)
        
        self.current_frame = None
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.current_frame = cv_image  # Store the latest frame

# Initialize ROS 2
rclpy.init()
subscriber_node = MinimalSubscriber()

# Flask web app for MJPEG streaming
flask_app = Flask(__name__)

@flask_app.route('/video_feed')
def video_feed():
    return Response(generate_mjpeg_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def generate_mjpeg_frames():
    """Generate MJPEG frames."""
    while True:
        if subscriber_node.current_frame is not None:
            ret, jpeg = cv2.imencode('.jpg', subscriber_node.current_frame)
            if not ret:
                continue
            frame = jpeg.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

# Dash web app
app = Dash(__name__, server=flask_app)

app.layout = html.Div([
    html.H1("Live Video Feed"),
    html.Img(src="/video_feed", style={"width": "80%", "height": "auto"})
])

# Function to spin ROS 2 in a separate thread
def run_ros2():
    rclpy.spin(subscriber_node)

def main():
    # Start the ROS 2 subscriber in a separate thread
    ros2_thread = threading.Thread(target=run_ros2)
    ros2_thread.start()

    # Run the Flask (which integrates with Dash)
    flask_app.run(debug=True, port=5000, host='0.0.0.0')

if __name__ == '__main__':
    main()
