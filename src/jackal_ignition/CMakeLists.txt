cmake_minimum_required(VERSION 3.0.2)
project(jackal_ignition)

set(CMAKE_CXX_STANDARD 14)

# required packages
set(RUNTIME_DEPS
  jackal_description
  ros_ign_gazebo
)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS ${RUNTIME_DEPS}
)

#############
## Install ##
#############

# Mark other files for installation (e.g. launch and bag files, etc.)
 install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

