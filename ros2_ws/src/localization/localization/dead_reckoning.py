import rclpy
from rclpy.node import Node
from cascade_msgs.msg import SensorReading
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf2_ros
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
import geometry_msgs.msg
import time
import math

class DeadReckoningNode(Node):
    def __init__(self):
        super().__init__ ("dead_reckoning_node")
        queue_size=20
        acceptable_delay=0.01 #this is how many seconds of difference we allow between the 2 subscriptions before theyre considered not matching
        tss = ApproximateTimeSynchronizer(
            [Subscriber(self, SensorReading, "/PID/yaw/actual"),
                Subscriber(self, SensorReading, "/PID/pitch/actual"),
                Subscriber(self, SensorReading, "/PID/roll/actual"),
                Subscriber(self, SensorReading, "/PID/surge/actual"),
                Subscriber(self, SensorReading, "/PID/sway/actual"),
                Subscriber(self, SensorReading, "/PID/heave/actual"),
            ],
            queue_size,
            acceptable_delay)
        tss.registerCallback(self.synced_callback)
        self.publisher_ = self.create_publisher(PoseStamped, '/pose', 10)
        self.current_x=0.0
        self.current_y=0.0
        self.current_z=0.0
        self.last_time = time.perf_counter()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
 
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def synced_callback(self, yaw_msg, pitch_msg, roll_msg, surge_msg, sway_msg, heave_msg):
        pose_msg=PoseStamped()

        pose_msg.header.stamp=self.get_clock().now().to_msg() 
        delta_t = time.perf_counter() - self.last_time; 
        self.last_time=time.perf_counter();

        quaternion = self.euler_to_quaternion(math.radians(roll_msg.data), math.radians(pitch_msg.data), math.radians(yaw_msg.data))

        # Create a TransformStamped message for the body-to-world transformation
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = pose_msg.header.stamp
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'base_link'  # Your body frame

        transform.transform.rotation = quaternion

        # Optionally, set the translation part if you need to track position
        transform.transform.translation.x = self.current_x
        transform.transform.translation.y = self.current_y
        transform.transform.translation.z = self.current_z
        
        br = tf2_ros.TransformBroadcaster(self)
        br.sendTransform(transform)
        
        vel_body = geometry_msgs.msg.Vector3()
        vel_body.x = surge_msg.data
        vel_body.y = sway_msg.data
        vel_body.z = heave_msg.data

        # Convert Vector3 to a stamped vector so that tf2 can handle it
        vel_body_stamped = geometry_msgs.msg.Vector3Stamped()
        vel_body_stamped.vector = vel_body
        vel_body_stamped.header.frame_id = 'base_link'
        vel_body_stamped.header.stamp = pose_msg.header.stamp

        # Transform velocity into the world frame
        try:
            # Transform the velocity to the world frame
            vel_world = self.tf_buffer.transform(vel_body_stamped, 'world', timeout=rclpy.duration.Duration(seconds=0.05))
            vel_world = vel_world_stamped.vector
            self.current_x += vel_world.x * delta_t
            self.current_y += vel_world.y * delta_t
            self.current_z += vel_world.z * delta_t
            self.get_logger().info(f'Transformed velocity: {transformed_vel.vector}')
        except Exception as ex:
            self.get_logger().warn(f'Failed to transform velocity: {str(ex)}')

        pose_msg.pose.position.x=self.current_x
        pose_msg.pose.position.y=self.current_y
        pose_msg.pose.position.z=self.current_z
        pose_msg.pose.orientation=quaternion

        self.publisher_.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
