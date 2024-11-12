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

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "got matching request");
        // convert our pointcloud to open3d type
    auto source = makeOpen3DPointCloud(request->reference);
    auto target = makeOpen3DPointCloud(request->actual);
    
    open3d::io::WritePointCloud("test_source.pcd", source, {false, false});
    open3d::io::WritePointCloud("test_target.pcd", target, {false, false});

    double search_radius = 5 * 0.02;
    int max_neighbours = 100;
    source.EstimateNormals(
            open3d::geometry::KDTreeSearchParamHybrid(2 * 0.02, 30));
    target.EstimateNormals(
            open3d::geometry::KDTreeSearchParamHybrid(2 * 0.02, 30));
    // get features
    auto source_feature = open3d::pipelines::registration::ComputeFPFHFeature(source, 
            open3d::geometry::KDTreeSearchParamHybrid(search_radius, max_neighbours)); // search radius, max neighbours
    auto target_feature = open3d::pipelines::registration::ComputeFPFHFeature(target, 
            open3d::geometry::KDTreeSearchParamHybrid(search_radius, max_neighbours));

    // fast global registration args
    double division_factor    = 1.4;
    bool   use_abs_scale      = true;
    bool   decrease_mu        = true;
    double distance_threshold = 0.02 * 1.5;
    int    max_iterations     = 100;
    double touple_scale       = 0.95;
    int    max_tuples        = 1000;

    //preform fast global registration
    open3d::pipelines::registration::RegistrationResult result =
        open3d::pipelines::registration::
        FastGlobalRegistrationBasedOnFeatureMatching(source, target, *source_feature, *target_feature,
                open3d::pipelines::registration::FastGlobalRegistrationOption(
                    division_factor,
                    use_abs_scale,
                    decrease_mu,
                    distance_threshold,
                    max_iterations,
                    touple_scale,
                    max_tuples));

     
    

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "fitness: %f",result.fitness_);
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
