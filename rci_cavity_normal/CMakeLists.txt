cmake_minimum_required(VERSION 2.8.3)
project(rci_cavity_normal)

add_definitions(-std=c++11 -O3)

set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  roscpp
  cmake_modules
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

find_package(Eigen REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###################################
## catkin specific configuration ##
###################################
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
#  LIBRARIES
  CATKIN_DEPENDS
  	roscpp
        cmake_modules
	nodelet
        message_filters
        sensor_msgs
        geometry_msgs
        rci_comm
	pcl_conversions
  	pcl_ros
  DEPENDS eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include/rci_cavity_normal
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

link_directories(
${catkin_LIBRARY_DIRS}
${PCL_LIBRARY_DIRS}
${Boost_LIBRARY_DIRS}
${Eigen_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
add_definitions(${Eigen_DEFINITIONS})

add_library(cavity_normal_nodelet src/cavity_normal_nodelet.cpp include/rci_cavity_normal/cavity_normal_nodelet.h)
add_dependencies(cavity_normal_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(cavity_normal_nodelet
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_executable(cavity_normal_node src/cavity_normal_node.cpp)
add_dependencies(cavity_normal_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(cavity_normal_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES}
)
