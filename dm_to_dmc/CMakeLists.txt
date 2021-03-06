cmake_minimum_required(VERSION 2.8.3)
project(dm_to_dmc)

set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  sensor_msgs
  cv_bridge
  image_transport
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV 2.4.8 REQUIRED)

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
	sensor_msgs 
        cv_bridge
        image_transport
  DEPENDS opencv
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include/dm_to_dmc
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIR}
)

link_directories(
${catkin_LIBRARY_DIRS}
${Boost_LIBRARY_DIRS}
${OpenCV_LIBRARY_DIRS}
)

add_library(dm32_to_dm_nodelet src/dm32_to_dm_nodelet.cpp include/dm_to_dmc/dm32_to_dm_nodelet.h)
add_dependencies(dm32_to_dm_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(dm32_to_dm_nodelet
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_executable(dm32_to_dm_node src/dm32_to_dm_node.cpp)
add_dependencies(dm32_to_dm_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(dm32_to_dm_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
)

