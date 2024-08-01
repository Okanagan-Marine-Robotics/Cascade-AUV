import tkinter as tk
import rclpy
from rclpy.node import Node
from cascade_msgs.msg import MotorThrottle
import time
import threading

class GUI(Node):
    def __init__(self, root):
        super().__init__('gui_node')  # Initialize ROS 2 node
        self.publisher = self.create_publisher(MotorThrottle, 'motor_throttle', 10)

        # Create and place labels for text boxes
        tk.Label(root, text="Duration (seconds):").grid(row=3, column=0, padx=5, pady=5, sticky='e')
        tk.Label(root, text="Throttle (%):").grid(row=3, column=2, padx=5, pady=5, sticky='e')

        # Create text boxes for time and throttle
        self.time_entry = tk.Entry(root, width=10)
        self.time_entry.grid(row=3, column=1, padx=5, pady=5)
        self.throttle_entry = tk.Entry(root, width=10)
        self.throttle_entry.grid(row=3, column=3, padx=5, pady=5)

        # Create buttons for movement
        tk.Button(root, text="Forward", command=self.move_forward, height=2, width=5).grid(row=0, column=1)
        tk.Button(root, text="Left", command=self.move_left, height=2, width=5).grid(row=1, column=0)
        tk.Button(root, text="Stop", command=self.stop, height=2, width=5, bg="red", font=("Helvetica", 12, "bold")).grid(row=1, column=1)
        tk.Button(root, text="Right", command=self.move_right, height=2, width=5).grid(row=1, column=2)
        tk.Button(root, text="Backward", command=self.move_backward, height=2, width=5).grid(row=2, column=1)
        
        # Additional buttons for up, down, turn clockwise, and turn counterclockwise
        tk.Button(root, text="Up", command=self.move_up, height=2, width=10).grid(row=0, column=2, padx=5)
        tk.Button(root, text="Down", command=self.move_down, height=2, width=10).grid(row=2, column=2, padx=5)
        tk.Button(root, text="Turn Clockwise", command=self.turn_clockwise, height=2, width=15).grid(row=1, column=3, pady=5)
        tk.Button(root, text="Turn Counterclockwise", command=self.turn_counterclockwise, height=2, width=20).grid(row=1, column=4, pady=5)
        self.end_time=0.0

    def move_up(self):
        self.execute_command("Move Up")

    def move_down(self):
        self.execute_command("Move Down")

    def move_left(self):
        self.execute_command("Move Left")

    def move_right(self):
        self.execute_command("Move Right")

    def move_forward(self):
        self.execute_command("Move Forward")

    def move_backward(self):
        self.execute_command("Move Backward")

    def turn_clockwise(self):
        self.execute_command("Turn Clockwise")

    def turn_counterclockwise(self):
        self.execute_command("Turn Counterclockwise")

    def stop(self):
        self.execute_command("Stop")

    def execute_command(self, command):
        duration=throttle=0.0
        try:
            duration = float(self.time_entry.get())
            throttle = float(self.throttle_entry.get())
            if(duration < 0.1):
                raise Exception()
        except Exception:
            duration=throttle=0.0
            self.get_logger().error("PLEASE ENTER A DURATION AND THROTTLE")
            return

        # Create and publish the MotorThrottle message

        msg = MotorThrottle()
        fli=flo=fri=fro=bli=blo=bri=bro=0.0

        if(command=="Move Right"):
            fro=bro=throttle
            flo=blo=-throttle
        if(command=="Move Left"):
            flo=blo=throttle
            fro=bro=-throttle
        if(command=="Move Forward"):
            fro=flo=throttle
            bro=blo=-throttle
        if(command=="Move Backward"):
            bro=blo=throttle
            fro=flo=-throttle

        if(command=="Move Up"):
            fri=bri=fli=bli=throttle
        if(command=="Move Down"):
            fri=bri=fli=bli=-throttle

        if(command=="Turn Clockwise"):
            fro=blo=throttle
            bro=flo=-throttle
        if(command=="Turn Counterclockwise"):
            fro=blo=-throttle
            bro=flo=throttle

        msg.fli=fli
        msg.fri=fri
        msg.flo=flo
        msg.fro=fro
        msg.bli=bli
        msg.bri=bri
        msg.blo=blo
        msg.bro=bro

        self.publisher.publish(msg)

        if(command != "Stop"):
            threading.Timer(duration, self.sendStop).start()

        self.end_time=time.perf_counter()+duration

        self.get_logger().info(f"{command} - Duration: {duration}s, Throttle: {throttle}%")

    def sendStop(self):
        if(time.perf_counter()>=self.end_time-0.1):
            self.publisher.publish(MotorThrottle())

def main():
    rclpy.init()  # Initialize ROS 2
    root = tk.Tk()
    root.title("Cascade Control GUI")

    gui = GUI(root)
    root.mainloop()

    # Run ROS 2 and Tkinter event loops
    gui.destroy_node()
    rclpy.shutdown()
    root.destroy()

if __name__ == "__main__":
    main()
