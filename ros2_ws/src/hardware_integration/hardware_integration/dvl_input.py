import struct
import socket


def receive_dvl_data(host: str, port: int):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect((host, port))
        print(f"Connected to {host}:{port}")

        try:
            while True:
                data = sock.recv(1024)  # Adjust the buffer size to 236 bytes
                if not data:
                    break
                print(data)
        except KeyboardInterrupt:
            print("Stopping data reception.")
        finally:
            print("Connection closed.")

if __name__ == "__main__":
    # Replace 'your_dvl_ip' and 'your_dvl_port' with the appropriate IP address and port
    receive_dvl_data('192.168.1.134', 9002)
