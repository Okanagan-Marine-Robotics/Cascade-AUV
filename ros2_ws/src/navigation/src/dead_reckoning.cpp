#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cascade_msgs/msg/sensor_reading.hpp"
#include "sensor_msgs/msg/imu.hpp"

typedef unsigned int uint32;
using std::placeholders::_1;

class DeadReckoningNode : public rclcpp::Node{
    public:
        float x,y,z,roll=0,pitch=0,yaw=0;
	float last_time;
	DeadReckoningNode() : Node("dead_reckoning_node") {
		gyro_subscription = this->create_subscription<sensor_msgs::msg::Imu>("/camera/gyro", 10, std::bind(&DeadReckoningNode::imu_callback, this, _1));
		roll_publisher = this->create_publisher<cascade_msgs::msg::SensorReading>("/roll", 10);
		pitch_publisher = this->create_publisher<cascade_msgs::msg::SensorReading>("/pitch", 10);
		yaw_publisher = this->create_publisher<cascade_msgs::msg::SensorReading>("/yaw", 10);
	}

	private:
	void imu_callback(const sensor_msgs::msg::Imu &msg) {
		float total_time = msg.header.stamp.sec + msg.header.stamp.nanosec * (1.0 / 1'000'000'000);
		float dt = total_time - last_time;
		last_time = total_time;

		roll  += msg.angular_velocity.x * dt;
		pitch += msg.angular_velocity.y * dt;
		yaw   += msg.angular_velocity.z * dt;

		auto roll_msg  = cascade_msgs::msg::SensorReading();
		auto pitch_msg = cascade_msgs::msg::SensorReading();
		auto yaw_msg   = cascade_msgs::msg::SensorReading();

        roll_msg.data=roll;
        yaw_msg.data=yaw;
        pitch_msg.data=pitch;

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
