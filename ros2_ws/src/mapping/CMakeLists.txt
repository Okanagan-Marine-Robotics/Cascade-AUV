cmake_minimum_required(VERSION 3.8)
project(mapping)

add_compile_options(-O3)
link_directories("$ENV{OPEN3D_ROOT}/lib")
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};$ENV{OPEN3D_ROOT}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cascade_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Open3D REQUIRED)

add_executable(mapping_node
 src/mapping_node.cpp
)

add_executable(matching_node
 src/matching.cpp
)

add_executable(conversion_node
 src/conversion_node.cpp
)

add_executable(depth_to_pointcloud
    src/depth_to_pointcloud.cpp
)
add_executable(visualizer 
    src/visualizer.cpp)

ament_target_dependencies(mapping_node rclcpp sensor_msgs geometry_msgs cv_bridge message_filters tf2 tf2_ros image_transport tf2_geometry_msgs OpenCV cascade_msgs)
target_include_directories(mapping_node PUBLIC include)
target_include_directories(mapping_node PUBLIC /usr/local/include/eigen3)

ament_target_dependencies(matching_node rclcpp sensor_msgs geometry_msgs cascade_msgs)
target_include_directories(matching_node PUBLIC include)
target_include_directories(matching_node PUBLIC /usr/local/include/eigen3)
target_link_libraries(matching_node Open3D Open3D::Open3D)

ament_target_dependencies(conversion_node rclcpp sensor_msgs cascade_msgs)
target_include_directories(conversion_node PUBLIC include)
target_include_directories(conversion_node PUBLIC /usr/local/include/eigen3)

ament_target_dependencies(depth_to_pointcloud rclcpp sensor_msgs geometry_msgs cv_bridge message_filters tf2 tf2_ros image_transport tf2_geometry_msgs OpenCV cascade_msgs OpenMP)
target_include_directories(depth_to_pointcloud PUBLIC include)
target_link_libraries(depth_to_pointcloud OpenMP::OpenMP_CXX)

ament_target_dependencies(visualizer rclcpp cascade_msgs glfw3 OpenGL tf2 tf2_geometry_msgs)
target_link_libraries(visualizer ${OPENGL_LIBRARIES} glfw)
target_include_directories(visualizer PUBLIC include)

install(TARGETS
    depth_to_pointcloud
    mapping_node
    conversion_node
    visualizer
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
