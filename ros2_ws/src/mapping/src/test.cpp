#include <iostream>
#include <random>
#include <vector>
#include <open3d/Open3D.h>
#include <Eigen/Core>
#include <string>

using namespace std;
using Eigen::Vector3d;

float voxel_size=0.02;

vector<Vector3d> load_pointcloud(string filename) {
    
}

vector<Vector3d> sphere_pointcloud(unsigned n, double radius) {
    
}

vector<Vector3d> square_pointcloud(double w, double l, double h) {
    vector<Vector3d> cloudData;
    for(float x=-w/2;x<=w/2;x+=voxel_size){
        for(float y=-l/2;y<=l/2;y+=voxel_resolution){
            for(float z=-h/2;z<=h/2;z+=voxel_resolution){
                if(abs(x)>=w-voxel_size*2 || abs(y)>=radius-voxel_resolution*2 || abs(z)>=radius-voxel_resolution*2)
                cloudData.push_back(voxelData(x,y,z,0, 100, 255,0,0));
            }
        }   
    }
    return cloudData;
}

open3d::geometry::PointCloud generatePool(){
    vector<Vector3d> cloudData;
    //make floor
    cloudData.append_range(square_pointcloud(10, 25, 0.03, vector(0,0,0)));
    //make walls
    cloudData.append_range(square_pointcloud(0.03, 25, 6, vector<float>(-5,0,3)));
    cloudData.append_range(square_pointcloud(0.03, 25, 6, vector<float>(5,0,3)));
    cloudData.append_range(square_pointcloud(10, 0.03, 6, vector<float>(0,-12.5,3)));
    cloudData.append_range(square_pointcloud(10, 0.03, 6, vector<float>(0,12.5,3)));
    //make gate
    cloudData.append_range(square_pointcloud(0.05, 0.05, 1, vector<float>(-1,0,3)));
    cloudData.append_range(square_pointcloud(0.05, 0.05, 1, vector<float>(1,0,3)));
    cloudData.append_range(square_pointcloud(2, 0.05, 0.05, vector<float>(0,0,3.5)));
    return open3d::geometry::PointCloud(cloudData);
}

open3d::pipelines::registration::RegistrationResult fgr(open3d::geometry::PointCloud source, open3d::geometry::PointCloud target){

    source.EstimateNormals(
            open3d::geometry::KDTreeSearchParamHybrid(2 * voxel_size, 30));
    target.EstimateNormals(
            open3d::geometry::KDTreeSearchParamHybrid(2 * voxel_size, 30));
    // get features
    auto source_feature = open3d::pipelines::registration::ComputeFPFHFeature(source, 
            open3d::geometry::KDTreeSearchParamHybrid(5 * voxel_size, 100)); // search radius, max neighbours
    auto target_feature = open3d::pipelines::registration::ComputeFPFHFeature(target, 
            open3d::geometry::KDTreeSearchParamHybrid(5 * voxel_size, 100));

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
    return result;
}

int main() {
    //generate or load 2 pointclouds
    generatePool();
    auto results = fgr(source,target);

    return 0;
}
