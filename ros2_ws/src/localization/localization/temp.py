import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped, Quaternion
import math
import tf2_ros
import tf2_geometry_msgs
 
class OrientationNode(Node):
    def __init__(self):
        super().__init__('orientation_node')
 
        # Initialize ROS2 subscribers and listeners
        self.subscription = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Initialize roll, pitch, yaw
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()
 
    def imu_callback(self, msg):
        # Calculate time delta
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
 
        # Angular velocity from IMU
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z
 
        # Integrate angular velocity to get Euler angles
        self.roll += wx * dt
        self.pitch += wy * dt
        self.yaw += wz * dt
 
        # Convert Euler angles to a quaternion
        quaternion_imu = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
 
        # Create a QuaternionStamped message
        orientation_imu = QuaternionStamped()
        orientation_imu.header.frame_id = 'imu_link'
        orientation_imu.quaternion = quaternion_imu
 
        try:
            # Look up the transform from imu_link to base_link
            transform = self.tf_buffer.lookup_transform('base_link', 'imu_link', rclpy.time.Time())
            # Transform the quaternion to the base frame
            orientation_base = tf2_geometry_msgs.do_transform_quaternion(orientation_imu, transform)
 
            # Log or use the transformed orientation as needed
            self.get_logger().info(f"Base frame orientation: {orientation_base.quaternion}")
        except tf2_ros.LookupException as e:
            self.get_logger().warn(f"Transform not available: {e}")
 
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
 
        return Quaternion(x=qx, y=qy, z=qz, w=qw)
 
def main(args=None):
    rclpy.init(args=args)
    node = OrientationNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
