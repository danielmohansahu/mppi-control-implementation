cmake_minimum_required(VERSION 3.0.2)
project(model_regression)

set(BUILD_DEPS
)
set(PKG_DEPS
  ${BUILD_DEPS}
)

## Find catkin macros and libraries
find_package(catkin REQUIRED ${BUILD_DEPS})

catkin_python_setup()

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
