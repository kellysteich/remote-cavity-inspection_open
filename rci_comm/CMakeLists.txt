cmake_minimum_required(VERSION 2.8.3)
project(rci_comm)

set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
)

###################################
## Msg and Srv ##
###################################
add_service_files(
  FILES
  InitSeedPoint.srv
  InitTwoSeedPoints.srv
  GetRealsenseParameters.srv
  GetFilterPipelineParams.srv
  GetTreeDetectionParams.srv
  GetBoxCheckParams.srv
  GetCavityNormalParams.srv
  GetKalmanParams.srv
  GetControllerParams.srv
  GetPdParams.srv
  SetFilterPipelineParams.srv
  SetTreeDetectionParams.srv
  SetBoxCheckParams.srv
  SetCavityNormalParams.srv
  SetKalmanParams.srv
  SetControllerParams.srv
  SetPdParams.srv
  LiftArm.srv
  StartController.srv
  ResetKalman.srv
)

add_message_files(
  FILES
  Tree.msg
  TreeAndCavities.msg
  TreeAndCavity.msg
  CavityBox.msg
  CavityBoxNormal.msg
  ArmMotorAngles.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
)

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
  CATKIN_DEPENDS 
	roscpp 
	sensor_msgs 
	std_msgs 
	message_runtime
)
