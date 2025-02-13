cmake_minimum_required(VERSION 3.0.2)
project(hydra_calibration)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_pkg
   CATKIN_DEPENDS
    rospy
    geometry_msgs
    moveit_commander
)

catkin_install_python(PROGRAMS
  scripts/calibration_routine.py
  scripts/check_calibration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
