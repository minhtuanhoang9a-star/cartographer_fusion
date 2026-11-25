cmake_minimum_required(VERSION 3.16)
project(cartographer_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visual_constraint_msgs REQUIRED)
find_package(cartographer REQUIRED)

add_library(cartographer_ros INTERFACE)
target_include_directories(cartographer_ros INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(cartographer_ros INTERFACE
  rclcpp::rclcpp tf2_ros::tf2_ros visual_constraint_msgs::visual_constraint_msgs
  cartographer::cartographer)

ament_export_dependencies(rclcpp tf2_ros visual_constraint_msgs cartographer)
ament_export_targets(cartographer_rosExport HAS_LIBRARY_TARGET)
install(DIRECTORY cartographer_ros/ DESTINATION include/cartographer_ros)
install(TARGETS cartographer_ros EXPORT cartographer_rosExport)
ament_package()
