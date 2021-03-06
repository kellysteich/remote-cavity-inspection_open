cmake_minimum_required(VERSION 2.8.3)
project(rci_controller)

add_definitions(-std=c++11 -O3)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
  roscpp
  nodelet
  cmake_modules
  eigen_conversions
  geometry_msgs
  nav_msgs
  sensor_msgs
  trajectory_msgs
  manipulator_msgs
  rci_comm
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

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
	nodelet
        cmake_modules
        eigen_conversions
  	geometry_msgs
  	nav_msgs
        sensor_msgs
        trajectory_msgs
        manipulator_msgs
        rci_comm
  DEPENDS eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include/rci_controller
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${Eigen_INCLUDE_DIRS}
)

link_directories(
${catkin_LIBRARY_DIRS}
${Boost_LIBRARY_DIRS}
${Eigen_LIBRARY_DIRS}
)

add_definitions(${Eigen_DEFINITIONS})

add_library(controller_nodelet src/controller_nodelet.cpp include/rci_controller/controller_nodelet.h)
add_dependencies(controller_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller_nodelet
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_executable(controller_node src/controller_node.cpp)
add_dependencies(controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
