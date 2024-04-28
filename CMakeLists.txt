cmake_minimum_required(VERSION 3.10) 
project(point_segmentation)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-std=c++17 -O3)

find_package(ament_cmake_auto)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/kitti_loader
  ${CMAKE_CURRENT_SOURCE_DIR}/src/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/utils
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  SYSTEM ${OpenCV_INCLUDE_DIRS}
)


ament_auto_find_build_dependencies()
ament_auto_add_executable(point_segmentation_node src/demo.cpp)

ament_auto_package(INSTALL_TO_SHARE launch)

#############
# QTCREATOR #
#############
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})
