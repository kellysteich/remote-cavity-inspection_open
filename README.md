# README: Remote Cavity Inspection#

This repository contains the code from my master thesis at ETHZ. The project is about remote cavity inspection (rci) using a drone with a robot arm with an attached stereo camera. In collaboration with ASL, DRZ and WSL.

### Dependencies ###
* ROS
* Qt
* Gazebo 6
* PCL
* OpenCv
* RotorS (https://github.com/ethz-asl/rotors_simulator)
* mav_comm_devel (private repository)
* mav_control (https://github.com/ethz-asl/mav_control)
* ethzasl_msf (https://github.com/ethz-asl/ethzasl_msf)
* ros_picoflexx (private repository)

### content description ###
* dm_to_dmc: helper node, converts between different depth image formats
* matlab: matlab scripts for testing and plotting
* rci_cavity_normal: node to get a normal for the cavity, by fitting a plane through surrounding points in the point cloud
* rci_comm: node with all the new messages and services for this project
* rci_controller: node that does the high level control of the drone, i.e positioning it in front of the cavity and inserting the arm into the cavity
* rci_description: description files needed in the simulator
* rci_filter_pipeline: node to filter the point cloud
* rci_gui : Gui for the project
* rci_img_tree_detection: node to detect a tree and cavity in a depth image
* rci_inverse_kinematics: node for the inverse kinematics of the robot arm
* rci_kalman: node for the kalman filter, that uses the cavity position detected by the vision part and the robot pose to output a continuous estimate of the cavity position
* rci_launch: node for all the launch files of the project
* rci_pc_cavity_check: node that refines the position estimate of the cavity by searching for a maximum fitting box in the point cloud starting from the output of rc_img_tree_detection
* rci_simulator: node to use the RotorS simulator for this project
* remote_cavity_inspection: node for the overall project




