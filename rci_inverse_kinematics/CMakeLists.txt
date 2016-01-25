cmake_minimum_required(VERSION 2.8.3)
project(rci_inverse_kinematics)

add_definitions(-std=c++11 -O3)

set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  nodelet
  geometry_msgs
  trajectory_msgs
  cmake_modules
  eigen_conversions
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
  	geometry_msgs
        trajectory_msgs
        eigen_conversions
        rci_comm
  DEPENDS eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include/rci_inverse_kinematics
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

add_library(inverse_kinematics_nodelet src/inverse_kinematics_nodelet.cpp include/rci_inverse_kinematics/inverse_kinematics_nodelet.h)
add_dependencies(inverse_kinematics_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(inverse_kinematics_nodelet
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_executable(inverse_kinematics_node src/inverse_kinematics_node.cpp)
add_dependencies(inverse_kinematics_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(inverse_kinematics_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
