import argparse
import rclpy
from rclpy.node import Node
from cascade_msgs.msg import SensorReading
from cascade_msgs.msg import MotorThrottle
import time
import serial

class MotorController(Node):
    def __init__(self):
        super().__init__('pid_cli_controller_node')
        self.publisher = self.create_publisher(MotorThrottle, 'motor_throttle', 10)
        self.pidPublisherMap={}

        self.pidPublisherMap["surge"] = self.create_publisher(SensorReading, "/PID/surge/target", 10)
        self.pidPublisherMap["sway"] = self.create_publisher(SensorReading, "/PID/sway/target", 10)
        self.pidPublisherMap["heave"] = self.create_publisher(SensorReading, "/PID/heave/target", 10)
        self.pidPublisherMap["roll"] = self.create_publisher(SensorReading, "/PID/roll/target", 10)
        self.pidPublisherMap["pitch"] = self.create_publisher(SensorReading, "/PID/pitch/target", 10)
        self.pidPublisherMap["yaw"] = self.create_publisher(SensorReading, "/PID/yaw/target", 10)
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("Serial port opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.serial_port = None

    def execute_command(self, commands, duration):
        try:
            duration = float(duration)
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
        if "Move Right" in commands:
            sway_msg.data=-commands['Move Right']
            self.get_logger().info("Moving Right")
        elif "Move Left" in commands:
            sway_msg.data=commands['Move Left']
            self.get_logger().info("Moving Left")
        if "Move Forward" in commands:
            surge_msg.data=commands['Move Forward']
            self.get_logger().info("Moving forward")
        elif "Move Backward" in commands:
            surge_msg.data=-commands['Move Backward']
            self.get_logger().info("Moving forward")
        if "Move Up" in commands:
            heave_msg.data=commands['Move Up']
            self.get_logger().info("Moving up")
        elif "Move Down" in commands:
            heave_msg.data=-commands['Move Down']
            self.get_logger().info("Moving down")
        if "Turn" in commands:
            yaw_msg.data=commands['Turn']
            self.get_logger().info("Turning")
        if "Roll" in commands:
            roll_msg.data=commands['Roll']
            self.get_logger().info("Rolling")
        if "Stop" in commands:
            self.get_logger().info("Stopping motors")

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

    def wait_for_pattern(self, pattern):
        buffer = ""
        while True:
            # Read one byte at a time
            byte = self.serial_port.read(1).decode('utf-8', errors='ignore')
            if not byte:
                continue
        
            buffer += byte

            # Check if pattern is in buffer
            if pattern in buffer:
                print(f"Pattern found: {pattern}")
                return buffer

            # Keep the buffer reasonably short to prevent memory issues
            if len(buffer) > len(pattern) * 2:
                buffer = buffer[-len(pattern):]

    def qualify(self):
        self.execute_command({"Stop":0.0}, 5)
        self.execute_command({"Move Down":0.3}, 5)
        self.execute_command({"Move Forward": 0.5}, 12)
        self.execute_command({"Roll": 90.0}, 2.5)
        self.execute_command({"Stop":0.0}, 3)
        self.execute_command({"Roll": 90.0}, 2.5)
        self.execute_command({"Stop":0.0}, 3)
        self.execute_command({"Turn": 2.0}, 2)
        self.execute_command({"Move Forward": 0.5}, 12)
        self.execute_command({"Turn": 0.8,"Move Forward": 1.0}, 35)
        self.execute_command({"Stop":0.0}, 3)
        #do cirlce around buoy
        for i in range(50):
            self.publisher.publish(MotorThrottle())  # Send empty message to stop motors
            time.sleep(0.02)

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    while True:
        motor_controller.wait_for_pattern("Motors Enabled: 1")
        motor_controller.qualify()

    motor_controller.qualify()

    #rclpy.spin_once(motor_controller, timeout_sec=args.duration+1)  # Let the command finish
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
