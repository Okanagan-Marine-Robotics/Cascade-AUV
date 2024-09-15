
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cascade_msgs.msg import SensorReading  # Import your custom message
from cv_bridge import CvBridge
import cv2
from flask import Flask, Response
import threading
from dash import Dash, html, dcc
import plotly.graph_objs as go
from dash.dependencies import Output, Input  # Correct import for Output and Input

# ROS 2 subscriber node for image and custom sensor data (SensorReading)
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        
        # Image subscription
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # Replace with your image topic
            self.image_callback,
            1)
        
        # Custom sensor (SensorReading) subscription
        self.sensor_subscription = self.create_subscription(
            SensorReading,
            '/PID/yaw/actual',  # Replace with your custom sensor topic
            self.sensor_callback,
            10)
        
        self.current_frame = None
        self.bridge = CvBridge()
        self.sensor_data = []  # Store the latest sensor reading

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_image = cv2.resize(cv_image, (640, 480))
        self.current_frame = cv_image  # Store the latest frame
    
    def sensor_callback(self, msg):
        # Extract the data from the custom sensor message
        self.sensor_data.append(msg.data)
        if(len(self.sensor_data)>2000):
            self.sensor_data.pop(0)

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
    html.H1("Cascade Web Interface"),
    html.Div([
        html.Img(src="/video_feed", style={"width": "auto%", "height": "40%"}),
        dcc.Graph(id='sensor-graph', animate=False),  # Add plotly graph for SensorReading data
        dcc.Interval(id='interval-component', interval=50, n_intervals=0)
    ])
])

# Function to update the graph
@app.callback(
    Output('sensor-graph', 'figure'),  # Corrected Output import
    [Input('interval-component', 'n_intervals')]  # Corrected Input import
)
def update_graph(n):
    sensor_data = subscriber_node.sensor_data
    fig = go.Figure(
        data=[
            go.Scatter(y=sensor_data, mode='lines', name='Sensor Data')
        ]
    )
    fig.update_layout(title="Custom Sensor Data",
            yaxis_title="Sensor Value",
        xaxis_title="Time",
        yaxis=dict(
            autorange=True,  # Enable auto-scaling on y-axis
            fixedrange=False  # Make sure it's not fixed, allows dynamic scaling
        ),
        xaxis=dict(
            autorange=True,  # Enable auto-scaling on x-axis if needed
            fixedrange=False
        ),)
    return fig

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

