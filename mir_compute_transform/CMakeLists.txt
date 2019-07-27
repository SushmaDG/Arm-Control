cmake_minimum_required(VERSION 2.8.3)
project(mir_compute_transform)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    tf
)

install(PROGRAMS
  ros/scripts/compute_transform_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
