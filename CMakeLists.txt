cmake_minimum_required(VERSION 2.8.3)
project(planning)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  intera_core_msgs
)

catkin_python_setup()


catkin_package(
  CATKIN_DEPENDS
  rospy
  intera_core_msgs
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY share/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/share
  USE_SOURCE_PERMISSIONS
)

