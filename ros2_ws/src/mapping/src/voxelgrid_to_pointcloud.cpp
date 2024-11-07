#include "rclcpp/rclcpp.hpp"
#include <opencv2/core/core.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cascade_msgs/msg/classes.hpp"
#include "cascade_msgs/srv/find_object.hpp"
#include "cascade_msgs/msg/voxel_grid.hpp"
#include <opencv2/core/core.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "bonxai/bonxai.hpp"
#include "bonxai/serialization.hpp"
#include "voxelData.hpp"
#include <sstream>

using namespace std;
using namespace cv_bridge;
shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<cascade_msgs::msg::VoxelGrid>::SharedPtr gridPublisher;
bool inserting=false;


void conversion_callback(const shared_ptr<cascade_msgs::srv::Vg2pc::Request> request,
                                        shared_ptr<cascade_msgs::srv::Vg2pc::Response> response)
{
    string serialized_data(request.data.begin(), request.data.end());

    istringstream ifile(serialized_data, ios::binary);

    char header[256];
    ifile.getline(header, 256);
    Bonxai::HeaderInfo info = Bonxai::GetHeaderInfo(header);
    auto grid=Bonxai::Deserialize<voxelData>(ifile, info);

    auto accessor = grid.createAccessor();//accessor for voxelgrid

    std::ostringstream ofile(std::ios::binary);//output stream for pointcloud

    auto voxel_lambda = [&x,&y,&z,&accessor, &grid, &total, &request](const voxelData& data, const Bonxai::CoordT& coord) {
        if(data.class_id==request->object_type){
            Bonxai::Point3D pos = grid.coordToPos(coord);
            x+=pos.x;
            y+=pos.y;
            z+=pos.z;
            total++;
            //replace current pose estimation and implement icp here
            //https://en.wikipedia.org/wiki/Iterative_closest_point
            //https://pointclouds.org/documentation/classpcl_1_1_iterative_closest_point.html
            //check out usage example ^^
            //can use pcl
            //can put pc matching in a seperate node
        }
    };
    grid.forEachCell(voxel_lambda);
    if(total>0){
        response->pose.position.x=x/total;
        response->pose.position.y=y/total;
        response->pose.position.z=z/total;
        response->exists=true;
    }
    else
        response->exists=false;
}

bool replaceCostmap(cascade_msgs::msg::VoxelGrid msg){
    if(loading || searching)return false;
    loading=true;
        costmap=move(g);
    loading=false;
    inserted=true;
    return true;
}


void pc_subscription_callback(const sensor_msgs::msg::PointCloud2 &pc_msg){
    //possibly add a queue for inserting the depth maps?
    insertDepthImage(pc_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    //creates node as shared pointer
    node = rclcpp::Node::make_shared("mapping_node");

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscription=
    node->create_subscription<sensor_msgs::msg::PointCloud2>("/pointcloud",10, &pc_subscription_callback);

    gridPublisher = node->create_publisher<cascade_msgs::msg::VoxelGrid>("/voxel_grid", 10);

    rclcpp::Service<cascade_msgs::srv::FindObject>::SharedPtr service=node->create_service<cascade_msgs::srv::FindObject>("find_object", &find_object_callback);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
