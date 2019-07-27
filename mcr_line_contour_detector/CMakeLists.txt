cmake_minimum_required(VERSION 2.8.3)
project(mcr_line_contour_detector)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  cv_bridge
  image_transport
  mcr_perception_msgs
  tf
  pcl_ros
  control_msgs
  trajectory_msgs
  geometry_msgs
  nav_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(    
  DEPENDS
    Eigen
)

include_directories(
  ros/include
  common/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


add_executable(line_contour_finder_node
  #ros/src/nodes/line_contour_detector_node.cpp
  ros/src/nodes/line_contour_finder_node.cpp
  ros/src/codes/line_contour_finder_ros.cpp
  common/src/line_contour_finder.cpp
)

target_link_libraries(line_contour_finder_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}
)

add_dependencies(line_contour_finder_node
 ${catkin_EXPORTED_TARGETS}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
