cmake_minimum_required(VERSION 3.0.2)
project(mppi_controller)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Werror)

set(BUILD_DEPS
  roscpp
  geometry_msgs
  nav_msgs
  dynamic_reconfigure
  actionlib
  actionlib_msgs
)
set(PKG_DEPS
  message_generation
  ${BUILD_DEPS}
)

find_package(catkin REQUIRED ${BUILD_DEPS})
find_package(Eigen3 REQUIRED)


############################
## action file generation ##
############################
add_action_files(
  DIRECTORY action
  FILES Waypoint.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs nav_msgs geometry_msgs std_msgs
)

####################################
## dynamic reconfigure generation ##
####################################
generate_dynamic_reconfigure_options(
  config/MPPIOptions.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS ${PKG_DEPS}
 DEPENDS EIGEN3
)

###########
## Build ##
###########

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node
  src/forward_model.cpp
  src/mppi.cpp
  src/mppi_controller_node.cpp
  src/mppi_server.cpp
  src/visualization.cpp
 )

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

