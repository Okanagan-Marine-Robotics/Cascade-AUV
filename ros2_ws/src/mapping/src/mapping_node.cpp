#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cascade_msgs/msg/classes.hpp"
#include "cascade_msgs/srv/find_object.hpp"
#include "cascade_msgs/srv/matching.hpp"
#include "cascade_msgs/srv/vg2pc.hpp"
#include "cascade_msgs/msg/voxel_grid.hpp"
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "bonxai/bonxai.hpp"
#include "bonxai/serialization.hpp"
#include "voxelData.hpp"
#include <sstream>

using namespace std;
std::shared_ptr<rclcpp::Node> node, clientNode;
rclcpp::Publisher<cascade_msgs::msg::VoxelGrid>::SharedPtr gridPublisher;
rclcpp::Client<cascade_msgs::srv::Vg2pc>::SharedPtr conversion_client;
rclcpp::Client<cascade_msgs::srv::Matching>::SharedPtr matching_client;
bool inserting=false;

double voxel_resolution = 0.02;
Bonxai::VoxelGrid<voxelData> grid( voxel_resolution );

void insertSphere(float radius){
    auto accessor = grid.createAccessor();
    vector<voxelData> cloudData;
    for(float x=-radius;x<=radius;x+=voxel_resolution){
        for(float y=-radius;y<=radius;y+=voxel_resolution){
            for(float z=-radius;z<=radius;z+=voxel_resolution){
                if(x*x + y*y + z*z < radius*radius)
                cloudData.push_back(voxelData(x,y,z,0, 100, 255,0,0));
            }
        }   
    }
    for(voxelData vd : cloudData){
        Bonxai::CoordT coord = grid.posToCoord(vd.x, vd.y, vd.z);
        accessor.setValue(coord, vd); // Set voxel value to class ID
    }
    //publishVoxelGrid();
}

void insertBox(float radius){
    auto accessor = grid.createAccessor();
    vector<voxelData> cloudData;
    for(float x=0;x<=radius;x+=voxel_resolution){
        for(float y=0;y<=radius;y+=voxel_resolution){
            for(float z=0;z<=radius;z+=voxel_resolution){
                //if(abs(x)=radius-voxel_resolution*2 || abs(y)>=radius-voxel_resolution*2 || abs(z)>=radius-voxel_resolution*2)
                cloudData.push_back(voxelData(x,y,z,0, 100, 255,0,0));
            }
        }   
    }
    for(voxelData vd : cloudData){
        Bonxai::CoordT coord = grid.posToCoord(vd.x, vd.y, vd.z);
        accessor.setValue(coord, vd); // Set voxel value to class ID
    }
    //publishVoxelGrid();
}

void deleteAllVoxels(){
    auto accessor= grid.createAccessor();
    auto lambda = [&accessor](const voxelData& data, const Bonxai::CoordT& coord) {
        accessor.setCellOff(coord);
    };
    grid.forEachCell(lambda);
}

cascade_msgs::srv::Vg2pc::Response sendConversionRequest (cascade_msgs::srv::Vg2pc::Request request){          
    auto request_ptr = std::make_shared<cascade_msgs::srv::Vg2pc::Request>(request);

    while (!matching_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the vg2pc service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "vg2pc service not available, waiting again...");
    }

    auto result = conversion_client->async_send_request(request_ptr);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(clientNode, result) !=
        rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call vg2pc conversion service");
    }
    return *result.get();
}

cascade_msgs::srv::Matching::Response sendMatchingRequest (cascade_msgs::srv::Matching::Request request){            
    auto request_ptr = std::make_shared<cascade_msgs::srv::Matching::Request>(request);

    while (!matching_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the matching service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "matching service not available, waiting again...");
    }

    auto result = matching_client->async_send_request(request_ptr);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(clientNode, result) !=
        rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call matching service");
    }
    return *result.get();
}

void find_object_callback(const std::shared_ptr<cascade_msgs::srv::FindObject::Request> request,
                                        std::shared_ptr<cascade_msgs::srv::FindObject::Response>      response)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "received find object request");

    std::ostringstream ofile(std::ios::binary);
    Bonxai::Serialize(ofile, grid);
    std::string s=ofile.str();
    std::vector<unsigned char> charVector(s.begin(), s.end());

    cascade_msgs::srv::Vg2pc::Request conversion_request;

    conversion_request.voxel_grid.data=charVector;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending conversion request");
    auto conversion_response = sendConversionRequest(conversion_request);

    //getting the voxelData pointcloud

    cascade_msgs::srv::Matching::Request matching_request;
    matching_request.actual = conversion_response.pointcloud;
    matching_request.reference = box.pointcloud;
    //eventually will be filling in the reference based on the find_object request type
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending matching request");
    auto matching_response = sendMatchingRequest(matching_request);

    response->pose = matching_response.pose;
}

void publishVoxelGrid(){
    std::ostringstream ofile(std::ios::binary);
    Bonxai::Serialize(ofile, grid);
    std::string s=ofile.str();
    
    cascade_msgs::msg::VoxelGrid gridMsg;

    std::vector<unsigned char> charVector(s.begin(), s.end());
    gridMsg.data=charVector;

    gridPublisher->publish(gridMsg);
}

void decayAllVoxels(){//finish this
    auto accessor= grid.createAccessor();
    auto lambda = [&accessor](const voxelData& data, const Bonxai::CoordT& coord) {
    if(accessor.value(coord)==nullptr)return;
    if(data.confidence<30){
        accessor.setCellOff(coord);
        return;
    }
    accessor.value(coord)->confidence *= 0.99;//decay TODO: turn into a parameter, maybe make it a formula based on time
    };
    grid.forEachCell(lambda);
}


void insertDepthImage(const sensor_msgs::msg::PointCloud2 pc) {
    if (inserting) return; // Return fail to insert
    auto accessor = grid.createAccessor();
    inserting = true; // Making sure we don't insert multiple depth scans at once

    decayAllVoxels();
    
    //deserializing data

    std::string serialized_data(pc.data.begin(), pc.data.end());
    std::istringstream ifile(serialized_data, std::ios::binary);
    vector<voxelData> cloudData;
    
    for(unsigned int i=0;i<pc.width*pc.height;i++){
        voxelData out;
        ifile.read(reinterpret_cast<char*>(&out), sizeof(voxelData));
        cloudData.push_back(out);
    }

    for(voxelData vd : cloudData){
        Bonxai::CoordT coord = grid.posToCoord(vd.x, vd.y, vd.z);
            if(accessor.value(coord)!=nullptr){//if there is somethign already in the current voxel
                if(vd.confidence >= (*accessor.value(coord)).confidence && (*accessor.value(coord)).class_id == 0)//only change the class if we are more confident in the new recognition
                    accessor.setValue(coord, vd); // Set voxel value to class ID
            }
            else
                accessor.setValue(coord, vd);//if there isnt anything in current voxel, insert the detected class and confidence
    }
    publishVoxelGrid();
    inserting = false;
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
    clientNode = rclcpp::Node::make_shared("_mapping_client");
    conversion_client=clientNode->create_client<cascade_msgs::srv::Vg2pc>("vg2pc");
    matching_client=clientNode->create_client<cascade_msgs::srv::Matching>("pointcloud_matching");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
