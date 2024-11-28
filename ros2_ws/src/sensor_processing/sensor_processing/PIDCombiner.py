import rclpy
from rclpy.node import Node
from cascade_msgs.msg import SensorReading
from cascade_msgs.msg import MotorThrottle
from message_filters import ApproximateTimeSynchronizer, Subscriber
from math import copysign

class PIDCombinerNode(Node):
    def __init__(self):
        super().__init__ ("PID_combiner_node")
        queue_size=20
        acceptable_delay=0.01 #this is how many seconds of difference we allow between the 2 subscriptions before theyre considered not matching
        tss = ApproximateTimeSynchronizer(
            [Subscriber(self, SensorReading, "/PID_correction/yaw"),
                Subscriber(self, SensorReading, "/PID_correction/pitch"),
                Subscriber(self, SensorReading, "/PID_correction/roll"),
                Subscriber(self, SensorReading, "/PID_correction/surge"),
                Subscriber(self, SensorReading, "/PID_correction/sway"),
                Subscriber(self, SensorReading, "/PID_correction/heave"),
            ],
            queue_size,
            acceptable_delay)
        tss.registerCallback(self.synced_callback)
        self.publisher_ = self.create_publisher(MotorThrottle, '/motor_throttle', 10)
        self.lower_bound = 9

    def mapFromTo(self,x,a,b,c,d):
        y=(x-a)/(b-a)*(d-c)+c
        return y

    def synced_callback(self, yaw_msg, pitch_msg, roll_msg, surge_msg, sway_msg, heave_msg):
        motor_msg=MotorThrottle()
        #adding up corresponding pid values for each motor
        
        def constrain(value, min_val, max_val):
            return max(min_val, min(value, max_val))

        motor_msg.fli = constrain(-heave_msg.data - roll_msg.data + pitch_msg.data, -100.0, 100.0)
        motor_msg.fri = -constrain(-heave_msg.data + roll_msg.data + pitch_msg.data, -100.0, 100.0)
        motor_msg.bli = -constrain(-heave_msg.data - roll_msg.data - pitch_msg.data, -100.0, 100.0)
        motor_msg.bri = constrain(-heave_msg.data + roll_msg.data - pitch_msg.data, -100.0, 100.0)
        motor_msg.flo = constrain(surge_msg.data - sway_msg.data - yaw_msg.data, -100.0, 100.0)
        motor_msg.fro = constrain(surge_msg.data + sway_msg.data + yaw_msg.data, -100.0, 100.0)
        motor_msg.blo = constrain(surge_msg.data + sway_msg.data - yaw_msg.data, -100.0, 100.0)
        motor_msg.bro = constrain(surge_msg.data - sway_msg.data + yaw_msg.data, -100.0, 100.0)

        motor_msg.fli = self.mapFromTo(motor_msg.fli, 0, copysign(100, motor_msg.fli), copysign(self.lower_bound, motor_msg.fli), copysign(45, motor_msg.fli))
        motor_msg.fri = self.mapFromTo(motor_msg.fri, 0, copysign(100, motor_msg.fri), copysign(self.lower_bound, motor_msg.fri), copysign(45, motor_msg.fri))
        motor_msg.bli = self.mapFromTo(motor_msg.bli, 0, copysign(100, motor_msg.bli), copysign(self.lower_bound, motor_msg.bli), copysign(45, motor_msg.bli))
        motor_msg.bri = self.mapFromTo(motor_msg.bri, 0, copysign(100, motor_msg.bri), copysign(self.lower_bound, motor_msg.bri), copysign(45, motor_msg.bri))
        motor_msg.flo = self.mapFromTo(motor_msg.flo, 0, copysign(100, motor_msg.flo), copysign(self.lower_bound, motor_msg.flo), copysign(45, motor_msg.flo))
        motor_msg.fro = self.mapFromTo(motor_msg.fro, 0, copysign(100, motor_msg.fro), copysign(self.lower_bound, motor_msg.fro), copysign(45, motor_msg.fro))
        motor_msg.blo = self.mapFromTo(motor_msg.blo, 0, copysign(100, motor_msg.blo), copysign(self.lower_bound, motor_msg.blo), copysign(45, motor_msg.blo))
        motor_msg.bro = self.mapFromTo(motor_msg.bro, 0, copysign(100, motor_msg.bro), copysign(self.lower_bound, motor_msg.bro), copysign(45, motor_msg.bro))

        self.publisher_.publish(motor_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDCombinerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
