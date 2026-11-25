cmake_minimum_required(VERSION 3.16)
project(orbslam3_core)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_library(orbslam3_core INTERFACE)
target_include_directories(orbslam3_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(orbslam3_core INTERFACE Eigen3::Eigen opencv_core)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS orbslam3_core EXPORT orbslam3_coreTargets)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  cmake/orbslam3_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/orbslam3_coreConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/orbslam3_core)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/orbslam3_coreConfigVersion.cmake
  COMPATIBILITY SameMajorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/orbslam3_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/orbslam3_coreConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/orbslam3_core)
install(EXPORT orbslam3_coreTargets
  FILE orbslam3_coreTargets.cmake
  NAMESPACE orbslam3_core::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/orbslam3_core)
