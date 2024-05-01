cmake_minimum_required(VERSION 3.0.2)
project(rviz-camera-manager)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

set(SRC_FILES
  scripts/main.py
  scripts/layout.py
)


include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  )

catkin_install_python(PROGRAMS ${SRC_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )