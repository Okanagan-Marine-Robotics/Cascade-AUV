#include "Move1m.h"
#include "MovementBaseClass.h"
#include <rclcpp/rclcpp.hpp>

Move1m::Move1m(const std::string& name, const BT::NodeConfiguration& config)
    : MovementBaseClass(name, "MOVE_RELATIVE") 
{
   
}

BT::NodeStatus Move1m::tick() {
    //RCLCPP_INFO(rclcpp::get_logger("Move1M"), "Executing Move1M behavior");


    BT::NodeStatus status = MovementBaseClass::tick();

    if (status == BT::NodeStatus::RUNNING) {
        return BT::NodeStatus::RUNNING;
    }

    return status;
}

void Move1m::set_command_data(cascade_msgs::msg::MovementCommand &msg) {
  
    geometry_msgs::msg::Vector3 forward_vector;
    forward_vector.x = 1.0;  // Move forward by 1 meter
    forward_vector.y = 0.0;
    forward_vector.z = 0.0;
    
    // no rotation
    geometry_msgs::msg::Vector3 rotation_vector;
    rotation_vector.x = 0.0;
    rotation_vector.y = 0.0;
    rotation_vector.z = 0.0;

    // Assign the data to the MovementCommand message
    msg.data0 = forward_vector;
    msg.data1 = rotation_vector;
}