cmake_minimum_required(VERSION 3.8)
project(automated_planning)

add_compile_options(-O3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cascade_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

add_executable(bt_executor src/main.cpp src/Move1m.cpp src/TurnCW.cpp)

target_include_directories(serialization_test PUBLIC include)

ament_target_dependencies(costmap_node rclcpp sensor_msgs geometry_msgs cv_bridge message_filters tf2 tf2_ros image_transport tf2_geometry_msgs OpenCV cascade_msgs)
target_include_directories(costmap_node PUBLIC include)

target_link_libraries(bt_executor
    behaviortree_cpp::behaviortree_cpp
)


target_include_directories(bt_executor PRIVATE include)


install(TARGETS
    depth_to_pointcloud
    mapping_node
    visualizer
    serialization_test
    costmap_node
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
