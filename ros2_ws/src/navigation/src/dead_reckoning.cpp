#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cascade_msgs/msg/sensor_reading.hpp"

typedef unsigned uint32;

//create node 
class DeadReckoningNode : public rclcpp::Node{
    public:
        float x,y,z,roll,pitch,yaw;
	float last_time;
        

	DeadReckoning() : Node("deadreckoning_node" {
		subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("/camera/gyro", 10, std::bind(&DeadReckoningNode::topic_callback, this, _1));
		roll_publisher = this->create_publisher<cascade_msgs::msg::SensorReadings()>("/roll", 10);
		pitch_publisher_ = this->create_publisher<cascade_msgs::msg::SensorReadings()>("/pitch", 10);
		yaw_publisher_ = this->create_publisher<cascade_msgs::msg::SensorReadings()>("/yaw", 10);
	}

	private:
	void topic_callback(const sensor_msgs::msg::Imu &msg) const {
		float total_time = msg.header.time.sec + msg.header.time.nanosec * (1.0 / 1'000'000'000);
		float dt = total_time - last_time;
		last_time = total_time;

		roll  += msg.angular_velocity.x * dt;
		pitch += msg.angular_velocity.y * dt;
		yaw   += msg.angular_velocity.z * dt;

		auto roll_msg  = cascade_msgs::msg::SensorReadings();
		auto pitch_msg = cascade_msgs::msg::SensorReadings();
		auto yaw_msg   = cascade_msgs::msg::SensorReadings();

		pitch_publisher_ -> publish(pitch_msg);
		roll_publisher_  -> publish(roll_msg);
		yaw_publisher_   -> publish(yaw_msg);
	}


	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
	rclcpp::Publisher<cascade_msgs::msg::String>::SharedPtr roll_publisher_;
	rclcpp::Publisher<cascade_msgs::msg::String>::SharedPtr pitch_publisher_;
	rclcpp::Publisher<cascade_msgs::msg::String>::SharedPtr yaw_publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeadReckoningNode>());
  rclcpp::shutdown();


  return 0;
}
