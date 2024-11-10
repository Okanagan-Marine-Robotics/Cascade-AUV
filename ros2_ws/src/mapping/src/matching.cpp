#include "rclcpp/rclcpp.hpp"
#include <open3d/Open3D.h>

using namespace std;

shared_ptr<rclcpp::Node> node;


void matching(const shared_ptr<cascade_msgs::srv::Matching::Request> request,
                                        shared_ptr<cascade_msgs::srv::Matching::Response> response) {
    sensor_msgs::msg::PointCloud2 reference = request->reference, actual = request->actual;   

    
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    node = rclcpp::Node::make_shared("matching_server");

    rclcpp::Service<cascade_msgs::srv::Matching>::SharedPtr service=node->create_service<cascade_msgs::srv::Matching>("Matching", &Matching);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
