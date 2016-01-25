cmake_minimum_required(VERSION 2.8.3)
project(rci_launch)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rci_comm
    rci_description
    dm_to_dmc
    rci_filter_pipeline
    rci_img_tree_detection
    rci_pc_cavity_check
    rci_cavity_normal
    rci_kalman
    rci_controller
    rci_inverse_kinematics
    rci_gui
    rci_simulator
)

include_directories(include ${catkin_INCLUDE_DIRS})
