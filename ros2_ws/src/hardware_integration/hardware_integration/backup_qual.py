import argparse
import rclpy
from rclpy.node import Node
from cascade_msgs.msg import MotorThrottle
import time
import serial

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        self.publisher = self.create_publisher(MotorThrottle, 'motor_throttle', 10)

        self.fli = self.flo = self.fri = self.fro = self.bli = self.blo = self.bri = self.bro = 0.0
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("Serial port opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.serial_port = None

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

        if command == "Move Right":
            self.flo = self.bro = throttle
            self.fro = self.blo = -throttle
        elif command == "Move Left":
            self.flo = self.bro = -throttle
            self.fro = self.blo = throttle
        elif command == "Move Forward":
            self.fro = self.flo = self.bro = self.blo = throttle
        elif command == "Move Backward":
            self.fro = self.flo = self.bro = self.blo = -throttle
        elif command == "Move Up":
            self.fri = self.bri = self.fli = self.bli = -throttle
        elif command == "Move Down":
            self.fri = self.bri = throttle - 0.125
            self.fli = self.bli = throttle + 0.25
        elif command == "Turn Clockwise":
            self.fro = self.bro = -throttle
            self.blo = self.flo = throttle
        elif command == "Turn Counterclockwise":
            self.fro = self.bro = throttle
            self.blo = self.flo = -throttle
        elif command == "Stop":
            self.get_logger().info("Stopping motors")
            self.fli = self.flo = self.fri = self.fro = self.bli = self.blo = self.bri = self.bro = 0.0
        else:
            self.get_logger().error(f"Unknown command: {command}")
            return

        msg.fli = self.fli
        msg.fri = self.fri
        msg.flo = self.flo
        msg.fro = self.fro
        msg.bli = self.bli
        msg.bri = self.bri
        msg.blo = self.blo
        msg.bro = self.bro

        self.publisher.publish(msg)

        self.get_logger().info(f"{command} - Duration: {duration}s, Throttle: {throttle}%")
        #time.sleep(duration)

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
        self.execute_command("Stop", 10 , 0)
        time.sleep(5)
        self.execute_command("Move Down", 5 , 20)
        time.sleep(8)
        self.execute_command("Move Down", 5 , 17.5)
        self.execute_command("Move Forward", 5 , 20)
        time.sleep(25)
        self.execute_command("Turn Clockwise", 15 , 20)
        time.sleep(15)
        self.execute_command("Stop", 10 , 0)
    

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    while True:
        motor_controller.wait_for_pattern("Motors Enabled: 1")
        motor_controller.qualify()

    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
