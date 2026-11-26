cmake_minimum_required(VERSION 3.16)
project(cartographer)

# Start Modify
# Build Cartographer core from source instead of exposing headers only.
# VisualConstraint adds a virtual method to PoseGraphInterface, so the core
# library and cartographer_ros must be compiled against the same headers to
# keep the vtable layout consistent.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
add_subdirectory(cartographer_src)
# End Modify
