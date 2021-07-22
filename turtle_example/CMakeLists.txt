cmake_minimum_required(VERSION 3.0.2)
project(turtle_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  turtlesim
)

catkin_python_setup()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS scripts/circler DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
