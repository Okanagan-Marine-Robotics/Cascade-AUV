#include "rclcpp/rclcpp.hpp"
#include "cascade_msgs/srv/vg2pc.hpp"
#include "cascade_msgs/msg/voxel_grid.hpp"
#include "bonxai/bonxai.hpp"
#include "bonxai/serialization.hpp"
#include "voxelData.hpp"
#include <sstream>

using namespace std;
shared_ptr<rclcpp::Node> node;

void conversion_callback(const shared_ptr<cascade_msgs::srv::Vg2pc::Request> request,
                                        shared_ptr<cascade_msgs::srv::Vg2pc::Response> response)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "got conversion request");
    string serialized_data(request->voxel_grid.data.begin(), request->voxel_grid.data.end());

    istringstream ifile(serialized_data, ios::binary);

    char header[256];
    ifile.getline(header, 256);
    Bonxai::HeaderInfo info = Bonxai::GetHeaderInfo(header);
    auto grid=Bonxai::Deserialize<voxelData>(ifile, info);

    ostringstream ofile(ios::binary);//output stream for pointcloud
    int point_num=0;
    size_t point_size = sizeof(voxelData);

    auto voxel_lambda = [&ofile, &point_num, &point_size](const voxelData& data, const Bonxai::CoordT& coord) {
        ofile.write(reinterpret_cast<const char*>(&data), point_size);
        point_num++;
    };//for each voxel in the grid, add it to the pointcloud

    grid.forEachCell(voxel_lambda);

    std::string s=ofile.str();
    std::vector<unsigned char> charVector(s.begin(), s.end());

    //cloudMsg.header.stamp TODO SET HEADER STAMP?
    response->pointcloud.header.frame_id = "world";     
    response->pointcloud.height=1;
    response->pointcloud.width=point_num;
    //cloudMsg.fields TODO FILL OUT POINTFIELD info
    response->pointcloud.point_step=point_size;
    response->pointcloud.row_step=point_size*point_num;
    response->pointcloud.data=charVector;
    response->pointcloud.is_dense=true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    node = rclcpp::Node::make_shared("mapping_conversion_server");

    rclcpp::Service<cascade_msgs::srv::Vg2pc>::SharedPtr service=node->create_service<cascade_msgs::srv::Vg2pc>("vg2pc", &conversion_callback);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
