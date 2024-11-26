#ifndef MOVEMENTBASECLASS_H
#define MOVEMENTBASECLASS_H

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/action_node.h>
#include <string>
#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs.msg/Vector3>
#include <cascade_msgs/srv/Status.hpp>
#include <cascade_msgs/msg/MovementCommand.msg>


class MovementBaseClass : public BT::SyncActionNode 
{
public:
  MovementBaseClass(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
{
  BT::PortsList ports;
  ports.insert({"command",BT::InputPort<std::string>("command")});
  return ports;
  };
}

    BT::NodeStatus tick() override {
      
      void setCommandData(your_package::msg::MovementCommand &msg);

  ~MovementBaseClass();
        return BT::NodeStatus::SUCCESS;
    };



private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<cascade_msgs::msg::MovementCommand>::SharedPtr publisher_;
    rclcpp::Client<cascade_msgs::srv::CascadeStatus>::SharedPtr client_;

    bool message_sent_;
    std::string command_; 

#endif // MOVEMENTBASECLASS_H
