cmake_minimum_required(VERSION 2.8.3)
project(rci_img_tree_detection)

add_definitions(-std=c++11 -O3)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  sensor_msgs
  std_msgs
  geometry_msgs
  rci_comm
  cv_bridge
  image_transport
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV 2.4.8 REQUIRED)

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
	sensor_msgs 
	std_msgs 
        geometry_msgs
        rci_comm
        cv_bridge
        image_transport
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include/rci_img_tree_detection
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIR}
)

link_directories(
${catkin_LIBRARY_DIRS}
${Boost_LIBRARY_DIRS}
${OpenCV_LIBRARY_DIRS}
)

add_library(tree_cavity_detection_kmeans_nodelet src/tree_cavity_detection_kmeans_nodelet.cpp include/rci_img_tree_detection/tree_cavity_detection_kmeans_nodelet.h)
add_dependencies(tree_cavity_detection_kmeans_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(tree_cavity_detection_kmeans_nodelet
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}

)
add_executable(tree_cavity_detection_kmeans_node src/tree_cavity_detection_kmeans_node.cpp)
add_dependencies(tree_cavity_detection_kmeans_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(tree_cavity_detection_kmeans_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
)

