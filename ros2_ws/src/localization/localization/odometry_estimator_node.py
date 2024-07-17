import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import TwistWithCovarianceStamped
from cascade_msgs.msg import MotorThrottle
import numpy as np
import math

class OdometryEstimatorNode(Node):
    def __init__(self):
        super().__init__('odometry_estimator_node')
        self.create_subscription(MotorThrottle, '/motor_throttle', self.motor_throttle_callback, 10)
        self.twist_publisher_ = self.create_publisher(TwistWithCovarianceStamped, '/estimated_twist', 10)
        self.mass = 10.0  # Mass of the AUV in kg
        self.inertia = np.eye(3)  # Simplified inertia matrix (identity matrix)
        self.drag_coefficient = np.array([0.1, 0.1, 0.1])  # Simplified drag coefficients for surge, sway, and heave

        self.NUM_MOTORS = 8
        motor_positions = np.array([
            [0.45, 0.2, 0],
            [0.45, -0.2, 0],
            [-0.45, 0.2, 0],
            [-0.45, -0.2, 0],
            [0.55, 0.3, 0],
            [0.55, -0.3, 0],
            [-0.55, 0.3, 0],
            [-0.55, -0.3, 0],
        ])

        # Define motor orientations based on the provided orientations
        angle_rad = math.radians(30)

        # Define motor orientations based on the specified rotations
        motor_orientations = np.array([
            [0, 0, 1],   # fli
            [0, 0, 1],   # fri
            [0, 0, 1],   # bli
            [0, 0, 1],   # bri
            [math.cos(angle_rad), math.sin(angle_rad), 0],   # flo
            [math.cos(-angle_rad), math.sin(-angle_rad), 0], # fro
            [math.cos(angle_rad), math.sin(angle_rad), 0],   # blo
            [math.cos(-angle_rad), math.sin(-angle_rad), 0]  # bro
        ])

        # Assign the updated arrays to self.motor_positions and self.motor_orientations
        self.motor_positions = motor_positions
        self.motor_orientations = motor_orientations

        self.previous_time = self.get_clock().now()
        self.linear_velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)

    def compute_forces_and_torques(self, throttles):
        forces = np.zeros(3)
        torques = np.zeros(3)
        
        for i in range(self.NUM_MOTORS):
            thrust = throttles[i] * 0.4  # Thrust is proportional to throttle
            force = thrust * self.motor_orientations[i]
            forces += force
            torque = np.cross(self.motor_positions[i], force)
            torques += torque
        
        return forces, torques

    def compute_twist(self, forces, torques, dt):
        # Include drag forces
        drag_force = -self.drag_coefficient * self.linear_velocity
        net_forces = forces + drag_force

        # Compute linear and angular accelerations
        linear_acceleration = net_forces / self.mass
        angular_acceleration = np.linalg.inv(self.inertia) @ torques

        # Integrate to get velocities
        self.linear_velocity += linear_acceleration * dt
        self.angular_velocity += angular_acceleration * dt

        return self.linear_velocity, self.angular_velocity

    def motor_throttle_callback(self, msg):
        # Extract motor throttles
        throttles = [
            msg.fli, msg.fri, msg.bli, msg.bri,
            msg.flo, msg.fro, msg.blo, msg.bro
        ]

        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).nanoseconds * 1e-9
        self.previous_time = current_time

        # Compute forces and torques from motor throttles
        forces, torques = self.compute_forces_and_torques(throttles)

        # Compute velocities (linear and angular) considering drag
        linear_velocity, angular_velocity = self.compute_twist(forces, torques, dt)

        # Publish the estimated twist with covariance
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.stamp = current_time.to_msg()
        twist_msg.header.frame_id = "base_link"

        twist_msg.twist.twist.linear.x = linear_velocity[0]
        twist_msg.twist.twist.linear.y = linear_velocity[1]
        twist_msg.twist.twist.linear.z = linear_velocity[2]
        twist_msg.twist.twist.angular.x = angular_velocity[0]
        twist_msg.twist.twist.angular.y = angular_velocity[1]
        twist_msg.twist.twist.angular.z = angular_velocity[2]

        # For simplicity, set a fixed covariance. Adjust these values as needed.
        twist_msg.twist.covariance = [0.1] * 36

        self.twist_publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
