#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cascade_msgs/msg/sensor_reading.hpp"

//create node 
class DeadReckoningNode : public rclcpp::Node{
    public:
        float x,y,z,roll,pitch,yaw;
        
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeadReckoningNode>());
  rclcpp::shutdown();
  return 0;
}
