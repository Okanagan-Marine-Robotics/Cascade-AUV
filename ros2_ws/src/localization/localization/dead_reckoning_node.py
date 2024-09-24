import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from cascade_msgs.msg import SensorReading
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import QuaternionStamped, Quaternion
import math
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
import time

class ImuToPoseNode(Node):
    def __init__(self):
        super().__init__('dead_reckoning_node')
        
        # Subscriber to the IMU gyro data topic

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(
            Imu,
            '/camera/camera/gyro/sample',
            self.imu_callback,
            qos_profile)
        
        self.pidPublisherMap={}
        self.pidPublisherMap["yaw"] = self.create_publisher(SensorReading, "/PID/yaw/actual", 10)
        self.pidPublisherMap["pitch"] = self.create_publisher(SensorReading, "/PID/pitch/actual", 10)
        self.pidPublisherMap["roll"] = self.create_publisher(SensorReading, "/PID/roll/actual", 10)

        # Initialize the orientation
        self.orientation = [0.0, 0.0, 0.0]  # roll, pitch, yaw

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.last_time = time.perf_counter()

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
 
        return Quaternion(x=qx, y=qy, z=qz, w=qw)
    
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x*(180./math.pi), pitch_y*(180./math.pi), yaw_z*(180./math.pi)

    def imu_callback(self, msg):
        # Update orientation based on angular velocity
        
        delta_t = time.perf_counter() - self.last_time; 
        self.last_time = time.perf_counter()
        
        roll_msg=SensorReading()
        pitch_msg=SensorReading()
        yaw_msg=SensorReading()
        nullMsg=SensorReading()

        _time=self.get_clock().now().to_msg()
        roll_msg.header.stamp=_time
        pitch_msg.header.stamp=_time
        yaw_msg.header.stamp=_time

        if(abs(msg.angular_velocity.x) > 0.005):
            self.orientation[0] += msg.angular_velocity.x * delta_t  # roll
        if(abs(msg.angular_velocity.y) > 0.005):
            self.orientation[1] += msg.angular_velocity.y * delta_t  # pitch
        if(abs(msg.angular_velocity.z) > 0.005):
            self.orientation[2] += msg.angular_velocity.z * delta_t  # yaw

        quaternion_imu = self.euler_to_quaternion(self.orientation[1],
                                                    self.orientation[0], 
                                                    self.orientation[2])
 
        # Create a QuaternionStamped message
        orientation_imu = QuaternionStamped()
        orientation_imu.header.frame_id = 'imu_link'
        orientation_imu.quaternion = quaternion_imu

        pose_in = PoseStamped()
        pose_in.header.frame_id = "imu_link";
        pose_in.header.stamp = _time;
        pose_in.pose.position.x = 0.0;         
        pose_in.pose.position.y = 0.0;
        pose_in.pose.position.z = 0.0;
        pose_in.pose.orientation = orientation_imu.quaternion;         
        
        roll=pitch=yaw=0

        try:
            orientation_base = self.tf_buffer.transform(pose_in, 'base_link').pose.orientation

            # Log or use the transformed orientation as needed
            roll,pitch,yaw = self.euler_from_quaternion(orientation_base.x, orientation_base.y,orientation_base.z,orientation_base.w)

        except tf2_ros.LookupException as e:
            self.get_logger().warn(f"Transform not available: {e}")
        
        roll_msg.data=roll
        pitch_msg.data=-pitch
        yaw_msg.data=-yaw

        '''
        old apporach using angular velocity PID control instead of angle

        roll_msg.data=msg.angular_velocity.z 
        pitch_msg.data=-msg.angular_velocity.x
        yaw_msg.data=-msg.angular_velocity.y
        '''

        self.pidPublisherMap["yaw"].publish(roll_msg);
        self.pidPublisherMap["pitch"].publish(pitch_msg);
        self.pidPublisherMap["roll"].publish(yaw_msg);
        
        #get dvl velocity

def main(args=None):
    rclpy.init(args=args)
    node = ImuToPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
