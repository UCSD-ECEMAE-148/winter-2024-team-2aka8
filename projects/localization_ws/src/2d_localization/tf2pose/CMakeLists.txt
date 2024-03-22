cmake_minimum_required(VERSION 2.8.3)
project(tf2pose)

# List C++ dependencies on ros packages
set( ROS_CXX_DEPENDENCIES
  rospy
  tf
  geometry_msgs)

# Find catkin and all required ROS components
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})

# Set include directories
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare info that other packages need to import library generated here
catkin_package()
catkin_package(
    CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
)


include_directories(${catkin_INCLUDE_DIR})

catkin_install_python(PROGRAMS scripts/tf2pose.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

