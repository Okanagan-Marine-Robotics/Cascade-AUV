import socket
import re
import rclpy
from rclpy.node import Node
from cascade_msgs.msg import SensorReading

class DvlDriverNode(Node):
    def __init__(self):
        super().__init__('dvl_driver')
        self.pidPublisherMap={}

        self.pidPublisherMap["surge"] = self.create_publisher(SensorReading, "/PID/surge/actual", 10)
        self.pidPublisherMap["sway"] = self.create_publisher(SensorReading, "/PID/sway/actual", 10)
        self.pidPublisherMap["heave"] = self.create_publisher(SensorReading, "/PID/heave/actual", 10)

    def udp_server(self):
        # Create a UDP socket
        sock_bottom = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Bind the socket to the host and port
        sock_bottom.bind(('0.0.0.0', 9010))

        while True:
            # Receive data from the client
            data, dvl_addr = sock_bottom.recvfrom(1024)  # buffer size is 1024 bytes
            data = data.decode('ASCII')
            #print(f"Received message: {} from {dvl_addr}")


            # Regular expression to match the desired fields
            pattern = r"VX=(-?\d+\.\d+),VY=(-?\d+\.\d+),VZ=(-?\d+\.\d+),.*D1=(-?\d+\.\d+),D2=(-?\d+\.\d+),D3=(-?\d+\.\d+),D4=(-?\d+\.\d+)"

            # Find the matches
            matches = re.search(pattern, data)

            if matches:
            # Extracting values
                vx = float(matches.group(1))
                vy = float(matches.group(2))
                vz = float(matches.group(3))
                depths = []
                depths.append(float(matches.group(4)))
                depths.append(float(matches.group(5)))
                depths.append(float(matches.group(6)))
                depths.append(float(matches.group(7)))

                depth_avg=0
                for d in depths:
                    depth_avg += d / 4.0

                counter = depth_sum = std_dev = 0

                for d in depths:
                    std_dev += abs(depth_avg - d) / 4.0

                for d in depths:
                    if(d - depth_avg > std_dev * 1.5):
                        counter+=1
                        depth_sum += d
                if(counter >0):
                    depth_avg = depth_sum / counter
                else:
                    depth_avg=-1.0

                #publish avg dist from bottom

                _time=self.get_clock().now().to_msg()
                surge_msg=SensorReading()
                sway_msg=SensorReading()
                heave_msg=SensorReading()
                
                surge_msg.header.stamp = _time
                sway_msg.header.stamp = _time
                heave_msg.header.stamp = _time
                
                if(vx > -32 and vy > -32 and vz > -32):
                    surge_msg.data = vx
                    sway_msg.data=-vy
                    heave_msg.data=-vz
                else:
                    surge_msg.data = 0.0
                    sway_msg.data=0.0
                    heave_msg.data=0.0

                self.pidPublisherMap["surge"].publish(surge_msg)
                self.pidPublisherMap["sway"].publish(sway_msg)
                self.pidPublisherMap["heave"].publish(heave_msg)

                # Output the extracted variables
                #print(f"VX: {vx}, VY: {vy}, VZ: {vz}")
                #print(f"D1: {d1}, D2: {d2}, D3: {d3}, D4: {d4}")
            else:
                print("No matches found.")

def main(args=None):
    rclpy.init(args=args)
    node = DvlDriverNode()
    node.udp_server()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
