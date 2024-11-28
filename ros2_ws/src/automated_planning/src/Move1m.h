#ifndef MOVE1M_H
#define MOVE1M_H

#include "MovementBaseClass.h"  
#include <cascade_msgs/msg/movement_command.msg>  
#include <geometry_msgs/msg/vector3>  

class Move1m : public MovementBaseClass {
public:
    Move1m(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("command")};
    }

    BT::NodeStatus tick() override;

    //void set_command_data(cascade_msgs::msg::MovementCommand &msg) override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<your_package::msg::MovementCommand>::SharedPtr publisher_;
    rclcpp::Client<your_package::srv::CascadeStatus>::SharedPtr client_;

    bool message_sent_;
    std::string command_;

};

#endif //MOVE1M_H
