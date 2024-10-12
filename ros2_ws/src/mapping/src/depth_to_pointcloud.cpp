#include "rclcpp/rclcpp.hpp"
#include <opencv2/core/core.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
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
#include <vector>

using namespace std;
using namespace cv_bridge;

const float MAX_DIST=6;
std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPublisher;

void projectDepthImage(const cascade_msgs::msg::ImageWithPose img) {
    cv::Mat depth_img = cv_bridge::toCvCopy(img.depth)->image;
    cv::Mat rgb_img = cv_bridge::toCvCopy(img.rgb)->image;
    cv::Mat label_img = cv_bridge::toCvCopy(img.label)->image;

    const int w = depth_img.cols;
    const int h = depth_img.rows;
    const double cx = 320; // TODO: Use parameters for camera projection info
    const double cy = 240; // 
    const double fx_inv = 1.0 / 389.770416259766;
    const double fy_inv = 1.0 / 389.770416259766;

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
    
    std::vector<voxelData> pointcloud(h * w);
    std::ostringstream ofile(std::ios::binary);
    
    int actual_points=0;
    for (int v = 0; v < h; v+=1) {
        for (int u = 0; u < w; u+=1) {
            float depth = depth_img.at<unsigned short>(v, u); // Extract depth
            int class_id = static_cast<int>(label_img.at<cv::Vec2f>(v, u)[0]); // Extract class from the first channel
            int confidence = static_cast<int>(label_img.at<cv::Vec2f>(v, u)[1]); // Extract confidence from the second channel
            unsigned char r = rgb_img.at<cv::Vec3b>(v, u)[2]; 
            unsigned char g = rgb_img.at<cv::Vec3b>(v, u)[1]; 
            unsigned char b = rgb_img.at<cv::Vec3b>(v, u)[0]; 
            float x = depth*0.001;//make this a parameter, although it shouldnt ever change 

            if (x > 0.0 && x < MAX_DIST) {  
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

                //pointcloud[v*w + u] = voxelData(x,y,z,class_id, confidence, r,g,b);
                voxelData vd = voxelData(x,y,z,class_id, confidence, r,g,b);
                ofile.write(reinterpret_cast<const char*>(&vd), sizeof(voxelData));
                actual_points++;
            }
        }  
    }
    //publishing the projected data
    std::string s=ofile.str();
    std::vector<unsigned char> charVector(s.begin(), s.end());

    sensor_msgs::msg::PointCloud2 cloudMsg;

    //cloudMsg.header.stamp TODO SET HEADER STAMP?
    cloudMsg.header.frame_id = "world";     
    cloudMsg.height=1;
    cloudMsg.width=actual_points;
    //cloudMsg.fields TODO FILL OUT POINTFIELD info
    cloudMsg.point_step=sizeof(voxelData);
    cloudMsg.row_step=sizeof(voxelData)*actual_points;
    cloudMsg.data=charVector;
    cloudMsg.is_dense=true;

    cloudPublisher->publish(cloudMsg);
}

void img_subscription_callback(const cascade_msgs::msg::ImageWithPose &img_msg){
    projectDepthImage(img_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("depth_to_pointcloud_node");

    rclcpp::Subscription<cascade_msgs::msg::ImageWithPose>::SharedPtr img_subscription=
        node->create_subscription<cascade_msgs::msg::ImageWithPose>("/semantic_depth_with_pose",10, &img_subscription_callback);

    cloudPublisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud", 10);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
