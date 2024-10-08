#include "rclcpp/rclcpp.hpp"
#include <opencv2/core/core.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cascade_msgs/msg/image_with_pose.hpp"
#include "cascade_msgs/msg/classes.hpp"
#include "cascade_msgs/srv/find_object.hpp"
#include "cascade_msgs/msg/voxel_grid.hpp"
#include <opencv2/core/core.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "bonxai/bonxai.hpp"
#include "bonxai/serialization.hpp"
#include "voxelData.hpp"
#include <sstream>

using namespace std;
using namespace cv_bridge;
using std::placeholders::_1;
using std::placeholders::_2;

const float MAX_DIST=100;
std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<cascade_msgs::msg::VoxelGrid>::SharedPtr gridPublisher;
bool inserting=false;

double voxel_resolution = 0.03;
Bonxai::VoxelGrid<voxelData> grid( voxel_resolution );


float depth_to_meters(float d){
    return d*0.001;
}

void find_object_callback(const std::shared_ptr<cascade_msgs::srv::FindObject::Request> request,
                                        std::shared_ptr<cascade_msgs::srv::FindObject::Response>      response)
{
    auto accessor = grid.createAccessor();
    float x,y,z;
    x=y=z=0;
    int total=0;
    auto voxel_lambda = [&x,&y,&z,&accessor, &grid, &total, &request](const voxelData& data, const Bonxai::CoordT& coord) {
        if(data.class_id==request->object_type){
            Bonxai::Point3D pos = grid.coordToPos(coord);
            x+=pos.x;
            y+=pos.y;
            z+=pos.z;
            total++;
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

void insertArtificialGate(float x, float y, float z, float width, float height){
    auto accessor = grid.createAccessor();
    for(float i=-width;i<width;i+=grid.resolution*0.8){
        Bonxai::CoordT coord = grid.posToCoord(x, y+i, -z-height);
        accessor.setValue(coord, {cascade_msgs::msg::Classes::GATE,100});
    }
    for(float j=-height;j<height;j+=grid.resolution*0.8){
        Bonxai::CoordT coord = grid.posToCoord(x, y+width, -z+j);
        accessor.setValue(coord, {cascade_msgs::msg::Classes::GATE,100});
        coord = grid.posToCoord(x, y-width, -z+j);
        accessor.setValue(coord, {cascade_msgs::msg::Classes::GATE,100});
    }
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
                if(data.class_id==0)
                    accessor.setValue(coord, {data.class_id,data.confidence*0.6});//decay TODO: turn into a parameter, maybe make it a formula based on time
                else
                    accessor.setValue(coord, {data.class_id,data.confidence*0.6});//decay TODO: turn into a parameter
            };
            grid.forEachCell(lambda);
        }

bool insertDepthImage(const cascade_msgs::msg::ImageWithPose img) {
    auto accessor = grid.createAccessor();
    if (inserting) return false; // Return fail to insert
    inserting = true; // Making sure we don't insert multiple depth scans at once

    decayAllVoxels();

    cv::Mat depth_img = cv_bridge::toCvCopy(img.image)->image;
    int w = depth_img.cols;
    int h = depth_img.rows;
    double cx = 320; // TODO: Use parameters for camera projection info
    double cy = 240; // 
    double fx_inv = 1.0 / 389.770416259766;
    double fy_inv = 1.0 / 389.770416259766;

    // Convert current pose to tf2 Transform
    tf2::Transform tf_current_pose;
    tf2::fromMsg(img.pose, tf_current_pose);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_current_pose.getRotation()).getRPY(roll, pitch, yaw);

    // Multiply pitch and roll by -1
    pitch *= -1.0;
    roll *= -1.0;

    // Create a quaternion from the modified Euler angles
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    // Create a new transform with the modified orientation
    tf_current_pose.setRotation(q);

    // Robot's rotation matrix
    tf2::Matrix3x3 tf_R(tf_current_pose.getRotation());

    for (int u = 0; u < w; ++u) {
        for (int v = 0; v < h; ++v) {
            float depth = depth_img.at<cv::Vec3f>(v, u)[0]; // Extract depth from the first channel
            int class_id = static_cast<int>(depth_img.at<cv::Vec3f>(v, u)[1]); // Extract class from the second channel
            int confidence = static_cast<int>(depth_img.at<cv::Vec3f>(v, u)[2]); // Extract confidence from the third channel
            //if(class_id==0)continue;

            float x = depth_to_meters(depth);
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "created mapping_server node! Value: %.2f", x);

            if (x > 0 && x < MAX_DIST) {  
                float y = -x * ((u - cx) * fx_inv); // Calculate real world projection of each pixel
                float z = x * ((v - cy) * fy_inv); // z is vertical, y is horizontal

                // Apply rotation to the point
                tf2::Vector3 rotated_point = tf_R * tf2::Vector3(x, y, z);
                x = rotated_point.x();
                y = rotated_point.y();
                z = rotated_point.z();

                // Translate the point according to robot's pose
                x += img.pose.position.x;
                y += img.pose.position.y;
                z -= img.pose.position.z;

                Bonxai::CoordT coord = grid.posToCoord(x, y, z);
                if(accessor.value(coord)!=nullptr){//if there is somethign already in the current voxel
                    if(confidence >= (*accessor.value(coord)).confidence && (*accessor.value(coord)).class_id == 0)//only change the class if we are more confident in the new recognition
                        accessor.setValue(coord, {class_id,confidence}); // Set voxel value to class ID
                }
                else
                    accessor.setValue(coord, {class_id,100.0});//if there isnt anything in current voxel, insert the detected class and confidence
            }
        }  
    }
    publishVoxelGrid();
    inserting = false;
    return true;
}

void img_subscription_callback(const cascade_msgs::msg::ImageWithPose &img_msg){
    //possibly add a queue for inserting the depth maps?
    insertDepthImage(img_msg);
    //insertArtificialGate(6,0,0,2.5,1);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    //creates node as shared pointer
    node = rclcpp::Node::make_shared("mapping_server");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
            "created mapping_server node!");

    rclcpp::Subscription<cascade_msgs::msg::ImageWithPose>::SharedPtr img_subscription=
    node->create_subscription<cascade_msgs::msg::ImageWithPose>("/semantic_depth_with_pose",10, &img_subscription_callback);

    gridPublisher = node->create_publisher<cascade_msgs::msg::VoxelGrid>("/voxel_grid", 10);

    rclcpp::Service<cascade_msgs::srv::FindObject>::SharedPtr service=node->create_service<cascade_msgs::srv::FindObject>("find_object", &find_object_callback);

    rclcpp::spin(node);
    rclcpp::shutdown();

    std::ofstream outputFile("map.bx", std::ios::binary);
    if (!outputFile.is_open()) {
        std::cerr << "Error: Unable to open file for writing" << std::endl;
        return 1;
    }

    //Bonxai::Serialize(outputFile, grid);
    outputFile.close();
    return 0;
}
