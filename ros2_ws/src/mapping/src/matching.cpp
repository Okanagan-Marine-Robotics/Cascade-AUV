#include "rclcpp/rclcpp.hpp"
#include <open3d/Open3D.h>
#include "cascade_msgs/srv/matching.hpp"
#include "voxelData.hpp"
#include <vector>

using namespace std;

shared_ptr<rclcpp::Node> node;

open3d::geometry::PointCloud makeOpen3DPointCloud(sensor_msgs::msg::PointCloud2 pc){
    std::string serialized_data(pc.data.begin(), pc.data.end());
    std::istringstream ifile(serialized_data, std::ios::binary);
    vector<Eigen::Vector3d> cloudData;
    
    for(unsigned int i=0;i<pc.width*pc.height;i++){
        voxelData out;
        ifile.read(reinterpret_cast<char*>(&out), sizeof(voxelData));
        cloudData.push_back(out.toEigen());
    }
    return open3d::geometry::PointCloud(cloudData);
}

void matching_callback(const shared_ptr<cascade_msgs::srv::Matching::Request> request,
                                        shared_ptr<cascade_msgs::srv::Matching::Response> response) {
    sensor_msgs::msg::PointCloud2 reference = request->reference, actual = request->actual;   
    //extracting the pointclouds from the request
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "got matching request");
    auto open3d_reference = makeOpen3DPointCloud(reference);
    auto open3d_actual = makeOpen3DPointCloud(actual);
    //run open3D fgr code   
    //visualize the 2 pointclouds
    open3d::io::WritePointCloud("test_box.pcd", open3d_actual, {false, false});
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    node = rclcpp::Node::make_shared("matching_server");

    rclcpp::Service<cascade_msgs::srv::Matching>::SharedPtr service=node->create_service<cascade_msgs::srv::Matching>("pointcloud_matching", &matching_callback);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
