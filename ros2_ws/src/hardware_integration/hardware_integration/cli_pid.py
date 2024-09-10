import argparse
import rclpy
from rclpy.node import Node
from cascade_msgs.msg import SensorReading
import time

class MotorController(Node):
    def __init__(self):
        super().__init__('pid_cli_controller_node')
        self.pidPublisherMap={}

        self.pidPublisherMap["surge"] = self.create_publisher(SensorReading, "/PID/surge/target", 10)
        self.pidPublisherMap["sway"] = self.create_publisher(SensorReading, "/PID/sway/target", 10)
        self.pidPublisherMap["heave"] = self.create_publisher(SensorReading, "/PID/heave/target", 10)
        self.pidPublisherMap["roll"] = self.create_publisher(SensorReading, "/PID/roll/target", 10)
        self.pidPublisherMap["pitch"] = self.create_publisher(SensorReading, "/PID/pitch/target", 10)
        self.pidPublisherMap["yaw"] = self.create_publisher(SensorReading, "/PID/yaw/target", 10)

    def execute_command(self, command, duration, speed):
        try:
            duration = float(duration)
            speed = float(speed)
            if duration < 0.1:
                raise ValueError("Duration must be at least 0.1 seconds.")
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        surge_msg = SensorReading()
        sway_msg = SensorReading()
        heave_msg = SensorReading()
        roll_msg = SensorReading()
        pitch_msg = SensorReading()
        yaw_msg = SensorReading()
        
        surge_msg.data = 0.0
        sway_msg.data = 0.0
        heave_msg.data = 0.0
        roll_msg.data = 0.0
        pitch_msg.data = 0.0
        yaw_msg.data = 0.0

        
        #publish zero to all pids except the intended ones 
        if command == "Move Right":
            sway_msg.data=-speed
        elif command == "Move Left":
            sway_msg.data=speed
        elif command == "Move Forward":
            surge_msg.data=speed
        elif command == "Move Backward":
            surge_msg.data=-speed
        elif command == "Move Up":
            heave_msg.data=speed
        elif command == "Move Down":
            heave_msg.data=-speed
        elif command == "Turn":
            yaw_msg.data=speed
        elif command == "Roll":
            roll_msg.data=speed
        elif command == "Stop":
            self.get_logger().info("Stopping motors")
        else:
            self.get_logger().error(f"Unknown command: {command}")
            return
        start = time.perf_counter()
        while(time.perf_counter() - start < duration ):
            _time=self.get_clock().now().to_msg()
            surge_msg.header.stamp = _time
            sway_msg.header.stamp = _time
            heave_msg.header.stamp = _time
            roll_msg.header.stamp = _time
            pitch_msg.header.stamp = _time
            yaw_msg.header.stamp = _time

            self.pidPublisherMap["surge"].publish(surge_msg)
            self.pidPublisherMap["sway"].publish(sway_msg)
            self.pidPublisherMap["heave"].publish(heave_msg)
            self.pidPublisherMap["roll"].publish(roll_msg)
            self.pidPublisherMap["pitch"].publish(pitch_msg)
            self.pidPublisherMap["yaw"].publish(yaw_msg)

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()

    parser = argparse.ArgumentParser(description='Control motors using ROS 2.')
    parser.add_argument('command', choices=[
        'Move Forward', 'Move Backward', 'Move Left', 'Move Right',
        'Move Up', 'Move Down','Turn', 'Roll' , 'Stop'
    ], help='The command to execute.')
    parser.add_argument('duration', type=float, help='Duration of the action in seconds.')
    parser.add_argument('speed', type=float, help='intended speed for the action.')

    args = parser.parse_args()

    motor_controller.execute_command(args.command, args.duration, args.speed)
    motor_controller.execute_command("Stop", args.duration, args.speed)

    #rclpy.spin_once(motor_controller, timeout_sec=args.duration+1)  # Let the command finish
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
