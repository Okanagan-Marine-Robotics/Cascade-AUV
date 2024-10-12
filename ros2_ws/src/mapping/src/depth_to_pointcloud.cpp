#include "rclcpp/rclcpp.hpp"
#include <opencv2/core/core.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud_2.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cascade_msgs/msg/image_with_pose.hpp"
#include <opencv2/core/core.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "voxelData.hpp"
#include <sstream>

using namespace std;
using namespace cv_bridge;
using std::placeholders::_1;
using std::placeholders::_2;

const float MAX_DIST=6;
std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPublisher;

float depth_to_meters(float d){
    return d*0.001;
}

void publishPointCloud(){
    std::ostringstream ofile(std::ios::binary);
    Bonxai::Serialize(ofile, grid);
    std::string s=ofile.str();
    
    cascade_msgs::msg::VoxelGrid gridMsg;

    std::vector<unsigned char> charVector(s.begin(), s.end());
    gridMsg.data=charVector;

    gridPublisher->publish(gridMsg);
}

void projectDepthImage(const cascade_msgs::msg::ImageWithPose img) {
    cv::Mat depth_img = cv_bridge::toCvCopy(img.depth)->image;
    cv::Mat rgb_img = cv_bridge::toCvCopy(img.rgb)->image;
    cv::Mat label_img = cv_bridge::toCvCopy(img.label)->image;

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

    // Multiply pitch and roll by -1 to correct
    pitch *= -1.0;
    roll *= -1.0;

    // Create a quaternion from the modified Euler angles
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    // Create a new transform with the modified orientation
    tf_current_pose.setRotation(q);

    // Robot's rotation matrix
    tf2::Matrix3x3 tf_R(tf_current_pose.getRotation());
    
    voxelData[][] pointcloud = voxelData[h][w];

    for (int u = 0; u < w; ++u) {
        for (int v = 0; v < h; ++v) {
            float depth = depth_img.at<cv::Vec3f>(v, u); // Extract depth
            int class_id = static_cast<int>(label_img.at<cv::Vec2f>(v, u)[0]); // Extract class from the first channel
            int confidence = static_cast<int>(label_img.at<cv::Vec2f>(v, u)[1]); // Extract confidence from the second channel
            unsigned char r = rgb_img.at<cv::Vec3b>(v, u)[0]; 
            unsigned char g = rgb_img.at<cv::Vec3b>(v, u)[1]; 
            unsigned char b = rgb_img.at<cv::Vec3b>(v, u)[2]; 
            float x = depth_to_meters(depth);

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

                pointcloud[u][v] = voxelData(x,y,z,class_id, confidence, r,g,b);
            }
        }  
    }
    publishPointCloud();
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
