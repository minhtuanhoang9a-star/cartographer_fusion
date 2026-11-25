cmake_minimum_required(VERSION 3.16)
project(orbslam3_ros2_wrapper)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visual_constraint_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(orbslam3_core REQUIRED)

include_directories(include)

add_library(orbslam3_ros2_wrapper INTERFACE)
target_include_directories(orbslam3_ros2_wrapper INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(orbslam3_ros2_wrapper INTERFACE
  rclcpp::rclcpp
  orbslam3_core)

ament_export_dependencies(
  rclcpp sensor_msgs geometry_msgs visual_constraint_msgs tf2_ros
  message_filters orbslam3_core)
ament_export_targets(orbslam3_ros2_wrapperExport HAS_LIBRARY_TARGET)
install(DIRECTORY include/
  DESTINATION include)
install(TARGETS orbslam3_ros2_wrapper
  EXPORT orbslam3_ros2_wrapperExport)
ament_package()