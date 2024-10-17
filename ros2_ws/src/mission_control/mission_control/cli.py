import argparse
import rclpy
from rclpy.node import Node
from cascade_msgs.msg import MotorThrottle
import time

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        self.publisher = self.create_publisher(MotorThrottle, 'motor_throttle', 10)

    def execute_command(self, command, duration, throttle):
        try:
            duration = float(duration)
            throttle = float(throttle)
            if duration < 0.1:
                raise ValueError("Duration must be at least 0.1 seconds.")
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        msg = MotorThrottle()
        fli = flo = fri = fro = bli = blo = bri = bro = 0.0

        if command == "Move Right":
            flo = bro = throttle
            fro = blo = -throttle
        elif command == "Move Left":
            flo = bro = -throttle
            fro = blo = throttle
        elif command == "Move Forward":
            fro = flo = bro = blo = throttle
        elif command == "Move Backward":
            fro = flo = bro = blo = -throttle
        elif command == "Move Up":
            fri = bri = fli = bli = -throttle
        elif command == "Move DF":
            fri = bri = fli = bli = throttle
            fro = flo = bro = blo = throttle
        elif command == "Move Down":
            fri = bri = fli = bli = throttle
        elif command == "Turn Clockwise":
            fro = blo = -throttle
            bro = flo = throttle
        elif command == "Turn Counterclockwise":
            fro = blo = throttle
            bro = flo = -throttle
        elif command == "Stop":
            for i in range(50):
                self.publisher.publish(MotorThrottle())  # Send empty message to stop motors
                time.sleep(0.02)
            return
        else:
            self.get_logger().error(f"Unknown command: {command}")
            return

        msg.fli = fli
        msg.fri = fri
        msg.flo = flo
        msg.fro = fro
        msg.bli = bli
        msg.bri = bri
        msg.blo = blo
        msg.bro = bro

        self.publisher.publish(msg)

        self.get_logger().info(f"{command} - Duration: {duration}s, Throttle: {throttle}%")

        if command != "Stop":
            time.sleep(duration)
            self.publisher.publish(MotorThrottle())  # Stop the motors

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()

    parser = argparse.ArgumentParser(description='Control motors using ROS 2.')
    parser.add_argument('command', choices=[
        'Move Forward', 'Move Backward', 'Move Left', 'Move Right',
        'Move Up', 'Move Down', 'Move DF','Turn Clockwise', 'Turn Counterclockwise', 'Stop'
    ], help='The command to execute.')
    parser.add_argument('duration', type=float, help='Duration of the action in seconds.')
    parser.add_argument('throttle', type=float, help='Throttle percentage for the action.')

    args = parser.parse_args()

    motor_controller.execute_command(args.command, args.duration, args.throttle)

    #rclpy.spin_once(motor_controller, timeout_sec=args.duration+1)  # Let the command finish
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
