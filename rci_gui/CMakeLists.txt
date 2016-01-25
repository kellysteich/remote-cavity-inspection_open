##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(rci_gui)

set(CMAKE_BUILD_TYPE Release)

find_package(PCL 1.7.2 REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV 2.4.8 REQUIRED)

find_package(VTK REQUIRED)
set (VTK_LIBRARIES vtkRendering vtkGraphics vtkHybrid QVTK)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS
  qt_build
  roscpp
  image_transport
  cv_bridge
  message_filters
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  mav_msgs
  gazebo_msgs
  rci_comm
  pcl_conversions
  pcl_ros
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/qcustomplot
)

link_directories(
${catkin_LIBRARY_DIRS}
${PCL_LIBRARY_DIRS}
${Boost_LIBRARY_DIRS}
${OpenCV_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package(
  CATKIN_DEPENDS
    qt_build
    roscpp
    image_transport
    cv_bridge
    message_filters
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    gazebo_msgs
    rci_comm
    pcl_conversions
    pcl_ros
)

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/rci_gui/*.hpp include/qcustomplot/*.h)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS
src/*.cpp
include/rci_gui/*.hpp
include/*.h
include/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(rci_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(rci_gui ${QT_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${VTK_LIBRARIES})
install(TARGETS rci_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

