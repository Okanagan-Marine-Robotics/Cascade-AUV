from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pid_config = os.path.join(
      'config',
      'pid.yaml'
    )

    return LaunchDescription([
        Node(
            package='object_detection',
            executable='example_detector',
        ),
        Node(
            package='hardware_integration',
            executable='dvl_dummy_driver',
        ),
        Node(
            package='object_detection',
            executable='image_data_merger',
            remappings=[
                ('/depth_map', '/camera/depth'),
            ],
        ),
        Node(
            package='mapping',
            executable='mapping_node',
        ),
        Node(
            package='mapping',
            executable='depth_to_pointcloud',
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            remappings=[
                ('/camera/camera/depth/image_rect_raw', '/camera/depth'),
                ('/camera/camera/color/image_raw', '/camera/rgb'),
            ],
        ),
        Node(
            package='mapping',
            executable='visualizer',
            remappings=[
                ('/path_grid', '/voxel_grid'),
            ],
        ),
       Node(
            package='navigation',
            executable='dead_reckoning',
        )    
       ])

