import rclpy
from rclpy.node import Node
from cascade_msgs.msg import MotorThrottle
import json
import serial  # Import pyserial

class SerialOutputNode(Node):
    def __init__(self):
        super().__init__('serial_output_node')

        # Initialize serial port (replace '/dev/ttyUSB0' with your ESP32 port and set the correct baud rate)
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info("Serial port opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.serial_port = None

        self.subscription = self.create_subscription(
            MotorThrottle,
            '/motor_throttle',
            self.throttle_callback,
            10
        )

    def throttle_callback(self, msg):
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().error("Serial port is not open.")
            return

        # Create a JSON object following the specified format
        json_msg = {
            "actuators": {
                "thrusters": [
                    {"speed": msg.fli, "id": 1},
                    {"speed": msg.fri, "id": 2},
                    {"speed": msg.bli, "id": 3},
                    {"speed": msg.bri, "id": 4},
                    {"speed": msg.flo, "id": 5},
                    {"speed": msg.fro, "id": 6},
                    {"speed": msg.blo, "id": 7},
                    {"speed": msg.bro, "id": 8}
                ],
                "servos": [
                    {"angle": 0, "id": 9}  # Example; adjust as needed
                ],
                "motors": []
            }
        }

        self.get_logger().info(f'{json_msg}')

        # Convert JSON object to string and send it over serial
        json_str = json.dumps(json_msg)
        try:
            self.serial_port.write((json_str + '\n').encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f"Error writing to serial port: {e}")

    def destroy_node(self):
        # Close serial port if open
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialOutputNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
