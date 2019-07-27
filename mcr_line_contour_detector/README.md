`roslaunch mcr_line_contour_detector line_contour_detector.launch`

`rostopic pub /mcr_perception/line_contour_detector/input/event_in std_msgs/String e_start`


Subscribe to the PoseArray topic in Rviz: `/mcr_perception/line_contour_detector/output/trajectory/original`

Play the bagfile
`rosbag play line_contour_finder_01.bag`

`rosbag play line_contour_finder_02.bag`
