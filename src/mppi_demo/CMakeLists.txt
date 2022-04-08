cmake_minimum_required(VERSION 3.0.2)
project(mppi_demo)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 17)

set(BUILD_DEPS
)
set(PKG_DEPS
  rospy
  geometry_msgs
  nav_msgs
  jackal_ignition
  mppi_controller
  rviz
  ${BUILD_DEPS}
)

## Find catkin macros and libraries
find_package(catkin REQUIRED ${BUILD_DEPS})

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS ${PKG_DEPS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

