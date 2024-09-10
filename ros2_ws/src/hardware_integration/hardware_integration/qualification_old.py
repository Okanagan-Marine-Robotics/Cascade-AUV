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

        self.global_offset = 0

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
            self.flo = self.bro = throttle + self.global_offset
            self.fro = self.blo = -throttle - self.global_offset
        elif command == "Move Left":
            self.flo = self.bro = -throttl - self.global_offsete
            self.fro = self.blo = throttle + self.global_offset
        elif command == "Move Forward":
            self.fro = self.flo = self.bro = self.blo = throttle + self.global_offset
            self.fro += 0.2
            self.bro += 0.2
            self.flo -= 0.2
            self.blo -= 0.2
        elif command == "Move Backward":
            self.fro = self.flo = self.bro = self.blo = -throttle - self.global_offset
        elif command == "Move Up":
            self.fri = self.bri = self.fli = self.bli = -throttle - self.global_offset
        elif command == "Move Down":
            self.fri = self.fli = throttle + 0.25 + self.global_offset
            self.bri = self.bli = throttle - 0.25 + self.global_offset
        elif command == "Turn Clockwise":
            self.fro = self.bro = -throttle - self.global_offset
            self.blo = self.flo = throttle + self.global_offset
        elif command == "Turn Counterclockwise":
            self.fro = self.bro = throttle
            self.blo = self.flo = -throttle
        elif command == "Barrel Clockwise":
            self.fri = self.bri = throttle
            self.bli = self.fli = -throttle
        elif command == "Barrel Counterclockwise":
            self.fri = self.bri = -throttle
            self.bli = self.fli = throttle
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
        self.execute_command("Move Down", 5 , 23)
        time.sleep(6.8)
        self.execute_command("Move Down", 5 , 17.75)
        self.execute_command("Move Forward", 5 , 20)
        time.sleep(22)#make this 22
        self.execute_command("Move Forward", 5 , 0)
        self.execute_command("Barrel Counterclockwise", 15 , 40)
        time.sleep(2.2)
        self.execute_command("Move Down", 5 , 19)
        time.sleep(2.5)
        self.execute_command("Barrel Counterclockwise", 15 , 40)
        time.sleep(2.2)
        self.execute_command("Move Down", 5 , 18)
        time.sleep(4)
        self.execute_command("Turn Counterclockwise", 5 , 22)
        time.sleep(2.3)
        self.execute_command("Move Down", 5 , 17.75)
        self.execute_command("Move Forward", 5 , 20)
        time.sleep(27)#make this 22
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
