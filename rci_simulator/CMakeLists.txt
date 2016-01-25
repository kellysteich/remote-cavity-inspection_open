cmake_minimum_required(VERSION 2.8.3)
project(rci_simulator)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS gazebo_msgs roscpp sensor_fusion_comm gazebo_ros)

catkin_package(
  CATKIN_DEPENDS
    gazebo_msgs
    roscpp
    sensor_fusion_comm
    gazebo_ros
    geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(start_msf src/start_msf.cpp)
target_link_libraries(start_msf ${catkin_LIBRARIES})
add_dependencies(start_msf ${catkin_EXPORTED_TARGETS})
