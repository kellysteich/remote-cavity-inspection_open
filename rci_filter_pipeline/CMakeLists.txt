cmake_minimum_required(VERSION 2.8.3)
project(rci_filter_pipeline)

add_definitions(-std=c++11 -O3)

set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  nodelet
  sensor_msgs
  rci_comm
  pcl_conversions
  pcl_ros
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

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
  CATKIN_DEPENDS
  	roscpp
	nodelet
	sensor_msgs 	
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
  include/rci_filter_pipeline
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


add_library(filter_pipeline_nodelet src/filter_pipeline_nodelet.cpp include/rci_filter_pipeline/filter_pipeline_nodelet.h)
add_dependencies(filter_pipeline_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(filter_pipeline_nodelet
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_executable(filter_pipeline_node src/filter_pipeline_node.cpp)
add_dependencies(filter_pipeline_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(filter_pipeline_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES}
)
