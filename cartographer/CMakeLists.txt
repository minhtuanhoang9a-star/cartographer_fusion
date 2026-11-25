cmake_minimum_required(VERSION 3.16)
project(cartographer)

include(GNUInstallDirs)

add_library(cartographer INTERFACE)
target_include_directories(cartographer INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>)
install(DIRECTORY cartographer/ DESTINATION include/cartographer)
install(TARGETS cartographer EXPORT cartographerTargets)
install(EXPORT cartographerTargets
  FILE cartographerTargets.cmake
  NAMESPACE cartographer::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/cartographer)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  cmake/cartographerConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/cartographerConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/cartographer)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/cartographerConfigVersion.cmake
  COMPATIBILITY SameMajorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/cartographerConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/cartographerConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/cartographer)
