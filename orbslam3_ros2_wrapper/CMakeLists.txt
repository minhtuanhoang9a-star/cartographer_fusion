cmake_minimum_required(VERSION 3.16)
project(orbslam3_ros2_wrapper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(USE_MOCK_ORBSLAM3 "Build wrapper against mock ORB-SLAM3 for testing" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visual_constraint_msgs REQUIRED)
# cv_bridge của ROS Humble được build với OpenCV 4.5.x ở /usr. Máy này còn
# OpenCV 4.6.0 ở /usr/local; cv_bridgeConfig.cmake tự gọi find_package(OpenCV)
# nên nếu không ép OpenCV_DIR trước, cả hai bản sẽ bị trộn vào link line.
set(_cv_bridge_prefixes ${CMAKE_PREFIX_PATH})
string(REPLACE ":" ";" _ament_prefixes "$ENV{AMENT_PREFIX_PATH}")
list(APPEND _cv_bridge_prefixes ${_ament_prefixes})
find_file(_cv_bridge_extras_file
  NAMES cv_bridge-extras.cmake
  PATHS ${_cv_bridge_prefixes}
  PATH_SUFFIXES share/cv_bridge/cmake
  NO_DEFAULT_PATH)
if(_cv_bridge_extras_file)
  include("${_cv_bridge_extras_file}")
  if(DEFINED OpenCV_CONFIG_PATH AND EXISTS "${OpenCV_CONFIG_PATH}/OpenCVConfig.cmake")
    set(OpenCV_DIR "${OpenCV_CONFIG_PATH}" CACHE PATH
        "OpenCV version used by cv_bridge" FORCE)
  endif()
endif()

find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

# cv_bridge đã find OpenCV; chỉ gọi lại nếu vì lý do nào đó nó chưa chạy.
if(NOT OpenCV_FOUND)
  find_package(OpenCV REQUIRED)
endif()
# Lưu ý real mode: nếu ORB-SLAM3 thật được build với OpenCV 4.6.0 ở /usr/local,
# cần rebuild ORB-SLAM3 với đúng bản OpenCV mà cv_bridge dùng (hoặc rebuild
# cv_bridge với 4.6.0) để tránh lệch ABI lúc chạy.
find_package(Eigen3 REQUIRED)

add_library(orbslam3_target INTERFACE)

if(USE_MOCK_ORBSLAM3)
  find_package(Sophus REQUIRED)
  target_include_directories(orbslam3_target INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/test/mock_orbslam3/include
  )
  target_link_libraries(orbslam3_target INTERFACE
    Sophus::Sophus
    Eigen3::Eigen
    ${OpenCV_LIBS}
  )
else()
  find_package(orbslam3_core REQUIRED)
  target_link_libraries(orbslam3_target INTERFACE
    orbslam3_core::orbslam3_core
  )
endif()

# Thư viện nội bộ dùng chung cho cả 3 node
add_library(constraint_bridge STATIC
  src/constraint_bridge.cpp
)
target_include_directories(constraint_bridge PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(constraint_bridge
  rclcpp
  builtin_interfaces
  visual_constraint_msgs
)
target_link_libraries(constraint_bridge
  orbslam3_target
  Eigen3::Eigen
)

# 3 node thực thi tương ứng 3 cấu hình cảm biến
foreach(node_name stereo_node rgbd_node mono_imu_node)
  add_executable(${node_name} src/${node_name}.cpp)
  target_include_directories(${node_name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(${node_name}
    rclcpp
    sensor_msgs
    geometry_msgs
    cv_bridge
    message_filters
    visual_constraint_msgs
  )
  target_link_libraries(${node_name}
    constraint_bridge
    orbslam3_target
    ${OpenCV_LIBS}
  )
  install(TARGETS ${node_name}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

ament_package()