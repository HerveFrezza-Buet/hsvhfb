cmake_minimum_required(VERSION 2.8.3)
project(hsvhfb)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  image_transport 
  cv_bridge
  dynamic_reconfigure
  message_generation
)

generate_dynamic_reconfigure_options(
  cfg/color.cfg
)

catkin_package()

install(PROGRAMS scripts/hsv_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
