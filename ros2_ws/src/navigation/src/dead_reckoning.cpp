#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cascade_msgs/msg/sensor_reading.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <chrono>

typedef unsigned int uint32;
using std::placeholders::_1;

class DeadReckoningNode : public rclcpp::Node{
    public:
        double x,y,z,roll=0,pitch=0,yaw=0;
        bool gotFirstTime=false;
        std::chrono::time_point<std::chrono::high_resolution_clock> last_time;
	    DeadReckoningNode() : Node("dead_reckoning_node") {
            rclcpp::QoS qos_profile(10);  // Create QoS profile with history depth 10
            qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

		    gyro_subscription = this->create_subscription<sensor_msgs::msg::Imu>("/camera/camera/gyro/sample", qos_profile, std::bind(&DeadReckoningNode::imu_callback, this, _1));
		    roll_publisher = this->create_publisher<cascade_msgs::msg::SensorReading>("PID/roll/actual", 10);
		    pitch_publisher = this->create_publisher<cascade_msgs::msg::SensorReading>("PID/pitch/actual", 10);
	    	yaw_publisher = this->create_publisher<cascade_msgs::msg::SensorReading>("PID/yaw/actual", 10);
	    }

	private:
	void imu_callback(const sensor_msgs::msg::Imu &msg) {
        if(!gotFirstTime){
            last_time = std::chrono::high_resolution_clock::now();
            gotFirstTime=true;
            return;
        }
	    //double total_time = msg.header.stamp.sec + msg.header.stamp.nanosec * (1.0 / 1'000'000'000);
		std::chrono::duration<double> duration = std::chrono::high_resolution_clock::now() - last_time;
        double dt = duration.count();
        last_time = std::chrono::high_resolution_clock::now();
		
		pitch  += msg.angular_velocity.x * dt;
		yaw += msg.angular_velocity.y * dt;
		roll   += msg.angular_velocity.z * dt;

		auto roll_msg  = cascade_msgs::msg::SensorReading();
		auto pitch_msg = cascade_msgs::msg::SensorReading();
		auto yaw_msg   = cascade_msgs::msg::SensorReading();

        roll_msg.data=roll*180.0/3.141592653589793238463;
        yaw_msg.data=yaw*180.0/3.141592653589793238463;
        pitch_msg.data=pitch*180.0/3.141592653589793238463;

        roll_msg.header=msg.header;
        pitch_msg.header=msg.header;
        yaw_msg.header=msg.header;

		pitch_publisher -> publish(pitch_msg);
		roll_publisher  -> publish(roll_msg);
		yaw_publisher   -> publish(yaw_msg);
	}
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gyro_subscription;
	rclcpp::Publisher<cascade_msgs::msg::SensorReading>::SharedPtr roll_publisher;
	rclcpp::Publisher<cascade_msgs::msg::SensorReading>::SharedPtr pitch_publisher;
	rclcpp::Publisher<cascade_msgs::msg::SensorReading>::SharedPtr yaw_publisher;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeadReckoningNode>());
  rclcpp::shutdown();


  return 0;
}
