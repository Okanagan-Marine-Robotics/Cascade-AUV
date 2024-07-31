#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <SerialPort.h> // Example for serial communication

#include "rclcpp/rclcpp.hpp"
#include "cascade_msgs/msg/motor_throttle.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class SerialOutputNode : public rclcpp::Node
{
public:
    SerialOutputNode() : Node("serial_output_node")
    {
        subscription_ = this->create_subscription<cascade_msgs::msg::MotorThrottle>(
            "/motor_throttle", 10, std::bind(&SerialOutputNode::throttle_callback, this, _1));

        // Initialize serial port (replace "COM3" with your ESP32 port and set the correct baud rate)
        serial_port_ = std::make_shared<SerialPort>("COM3", 9600);
        if (serial_port_->isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
        }
    }

private:
    void throttle_callback(const cascade_msgs::msg::MotorThrottle &msg)
    {
        // Create a JSON object following the specified format
        nlohmann::json json_msg = {
            {"actuators", {
                {"thrusters", {
                    {{"speed", msg.fli}, {"id", 1}},
                    {{"speed", msg.fri}, {"id", 2}},
                    {{"speed", msg.bli}, {"id", 3}},
                    {{"speed", msg.bri}, {"id", 4}},
                    {{"speed", msg.flo}, {"id", 5}},
                    {{"speed", msg.fro}, {"id", 6}},
                    {{"speed", msg.blo}, {"id", 7}},
                    {{"speed", msg.bro}, {"id", 8}}
                }},
                {"servos", {
                    {{"angle", 0}, {"id", 9}} // Example; adjust as needed
                }},
                {"motors", nlohmann::json::array()} // Empty array for motors
            }}
        };

        // Convert JSON object to string
        std::string json_str = json_msg.dump();

        // Send JSON string over serial
        if (serial_port_->isOpen()) {
            serial_port_->writeString(json_str + "\n");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Serial port is not open.");
        }
    }

    rclcpp::Subscription<cascade_msgs::msg::MotorThrottle>::SharedPtr subscription_;
    std::shared_ptr<SerialPort> serial_port_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialOutputNode>());
    rclcpp::shutdown();
    return 0;
}
