cmake_minimum_required(VERSION 2.8.3)
project(rci_pc_cavity_check)

add_definitions(-std=c++11 -O3)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  message_filters
  sensor_msgs
  geometry_msgs
  rci_comm
  pcl_conversions
  pcl_ros
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(PCL REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS 
    roscpp
    nodelet
    message_filters
    sensor_msgs
    geometry_msgs
    rci_comm
    pcl_conversions
    pcl_ros
#  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include/rci_pc_cavity_check
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

link_directories(
${catkin_LIBRARY_DIRS}
${PCL_LIBRARY_DIRS}
${Boost_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_library(box_check_nodelet src/box_check_nodelet.cpp include/rci_pc_cavity_check/box_check_nodelet.h)
add_dependencies(box_check_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(box_check_nodelet
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_executable(box_check_node src/box_check_node.cpp)
add_dependencies(box_check_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(box_check_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES}
)
