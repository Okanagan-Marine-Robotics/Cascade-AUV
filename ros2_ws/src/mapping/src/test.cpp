#include <iostream>
#include <random>
#include <vector>
#include <open3d/Open3D.h>
#include <Eigen/Core>
#include <string>

using namespace std;
using Eigen::Vector3d;

float voxel_size=0.04;

vector<Vector3d> load_pointcloud(string filename) {
    
}

vector<Vector3d> sphere_pointcloud(unsigned n, double radius) {
    
}

vector<Vector3d> square_pointcloud(float w, float l, float h, float xd, float yd, float zd) {
    vector<Vector3d> cloudData;
    for(float x=-w/2;x<=w/2;x+=voxel_size){
        for(float y=-l/2;y<=l/2;y+=voxel_size){
            for(float z=-h/2;z<=h/2;z+=voxel_size){
                if(abs(x)>=w/2-voxel_size*2 || abs(y)>=l/2-voxel_size*2 || abs(z)>=h/2-voxel_size*2)
                cloudData.push_back(Vector3d(x+xd,y+yd,z+zd));
            }
        }   
    }
    return cloudData;
}

vector<Vector3d> noise_pointcloud(float w, float l, float h, float xd, float yd, float zd, int sparseness) {
    vector<Vector3d> cloudData;
    for(float x=-w/2;x<=w/2;x+=voxel_size){
        for(float y=-l/2;y<=l/2;y+=voxel_size){
            for(float z=-h/2;z<=h/2;z+=voxel_size){
                int random = rand() % sparseness;
                if(random == sparseness/2)
                cloudData.push_back(Vector3d(x+xd,y+yd,z+zd));
            }
        }   
    }
    return cloudData;
}
void stretch(vector<Vector3d> &vec, float factor){//adds noise to each point in a pointcloud
    for(Vector3d &v: vec){
            v *= factor;
        
    }
}

void noiseUp(vector<Vector3d> &vec, int max_dist, int frequency){//adds noise to each point in a pointcloud
    for(Vector3d &v: vec){
        if(rand() % 100 > 100 - frequency){//randomly @ frequency
            auto random = Vector3d((rand()%max_dist - max_dist/2) / 100.0,(rand()%max_dist - max_dist/2) / 100.0,(rand()%max_dist - max_dist/2) / 100.0);//move the point up to 5cm
            v += random;
        }
    }
}

open3d::geometry::PointCloud generatePool(){
    vector<Vector3d> cloudData;
    //make floor
    auto temp = square_pointcloud(2.5, 5, 0.03, 0,0,0);
    cloudData.insert(cloudData.begin(), temp.begin(), temp.end());
    //make walls
    temp = square_pointcloud(0.03, 5, 2, -1.25,0,1);
    cloudData.insert(cloudData.begin(), temp.begin(), temp.end());
    temp = square_pointcloud(0.03, 5, 2, 1.25,0,1);
    cloudData.insert(cloudData.begin(), temp.begin(), temp.end());
    temp = square_pointcloud(2.5, 0.03, 2, 0,-2.5,1);
    cloudData.insert(cloudData.begin(), temp.begin(), temp.end());
    temp = square_pointcloud(2.5, 0.03, 2, 0,2.5,1);
    cloudData.insert(cloudData.begin(), temp.begin(), temp.end());
    square_pointcloud(2.5, 0.03, 2, 0,2.5,1);
    cloudData.insert(cloudData.begin(), temp.begin(), temp.end());
    noiseUp(cloudData,10,10);
    stretch(cloudData,2);
    //make gate
    temp = square_pointcloud(0.05, 0.05, 1, -1,0,1+1);
    cloudData.insert(cloudData.begin(), temp.begin(), temp.end());
    temp = square_pointcloud(0.05, 0.05, 1, 1,0,1+1);
    cloudData.insert(cloudData.begin(), temp.begin(), temp.end());
    temp = square_pointcloud(2, 0.05, 0.05, 0,0,1.5+1);
    cloudData.insert(cloudData.begin(), temp.begin(), temp.end()); 

    temp = square_pointcloud(0.05, 0.05, 1, -2+5,0,1);
    cloudData.insert(cloudData.begin(), temp.begin(), temp.end());
    temp = square_pointcloud(0.05, 0.05, 1, 2+5,0,1);
    cloudData.insert(cloudData.begin(), temp.begin(), temp.end());
    temp = square_pointcloud(4, 0.05, 0.05, 0+5,0,1.5);
    cloudData.insert(cloudData.begin(), temp.begin(), temp.end());

    noiseUp(cloudData,4,5);

    //temp = noise_pointcloud(5, 5, 5, 0,0,0,1000);
    //cloudData.insert(cloudData.begin(), temp.begin(), temp.end());
    return open3d::geometry::PointCloud(cloudData);
}



open3d::geometry::PointCloud generateGate(float x, float y, float z){
    vector<Vector3d> cloudData;
    //make gate
    auto temp = square_pointcloud(0.05, 0.05, 1, -1+x,0+y,3+z);
    cloudData.insert(cloudData.begin(), temp.begin(), temp.end());
    temp = square_pointcloud(0.05, 0.05, 1, 1+x,0+y,3+z);
    cloudData.insert(cloudData.begin(), temp.begin(), temp.end());
    temp = square_pointcloud(2, 0.05, 0.05, 0+x,0+y,3.5+z);
    cloudData.insert(cloudData.begin(), temp.begin(), temp.end());
    return open3d::geometry::PointCloud(cloudData);
}
void VisualizeRegistration(const open3d::geometry::PointCloud &source,
                           const open3d::geometry::PointCloud &target,
                           const Eigen::Matrix4d &Transformation) {
    std::shared_ptr<open3d::geometry::PointCloud> source_transformed_ptr(
            new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> target_ptr(new open3d::geometry::PointCloud);
    *source_transformed_ptr = source;
    *target_ptr = target;
    source_transformed_ptr->Transform(Transformation);
    open3d::visualization::DrawGeometries({source_transformed_ptr, target_ptr},
                                  "Registration result");
}
open3d::pipelines::registration::RegistrationResult fgr(open3d::geometry::PointCloud &source, open3d::geometry::PointCloud &target){
    //source = *source.VoxelDownSample(voxel_size);
    //target = *target.VoxelDownSample(voxel_size);
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
    double distance_threshold = voxel_size * 1;
    int    max_iterations     = 200;
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
    srand (time(NULL));
    auto target = generatePool();
    auto source = generateGate(0,50,3);
    auto results = fgr(source,target);
    open3d::io::WritePointCloud("test_source.pcd", source, {false, false});
    open3d::io::WritePointCloud("test_target.pcd", target, {false, false});
    VisualizeRegistration(source, target,results.transformation_);
    
    cout << results.fitness_ << '\n';

    return 0;
}
