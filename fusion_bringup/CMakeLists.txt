cmake_minimum_required(VERSION 3.16)
project(fusion_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch scripts
  DESTINATION share/${PROJECT_NAME})

ament_package()
