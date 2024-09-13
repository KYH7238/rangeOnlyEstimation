cmake_minimum_required(VERSION 3.0.2)
project(relative)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  rospy
  std_msgs
  rviz
  tf
  urdf
  xacro
  sensor_msgs
  message_generation
)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

add_message_files(
  FILES
  UwbRange.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs 
)

catkin_package(
   CATKIN_DEPENDS mavros_msgs roscpp rospy std_msgs message_runtime
)
catkin_install_python(PROGRAMS scripts/moving.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/uwb_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
)

# add_executable(relative src/test.cpp)
# target_link_libraries(relative ${catkin_LIBRARIES})
