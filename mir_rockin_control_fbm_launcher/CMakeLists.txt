cmake_minimum_required(VERSION 2.8.3)
project(mir_rockin_control_fbm_launcher)
find_package(catkin REQUIRED
  COMPONENTS
    rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
)


install(PROGRAMS
  ros/scripts/linear_interpolator_demo_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)