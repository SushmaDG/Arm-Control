## Launching rockin control fbm
### Launch robot components

	roscore

	roslaunch mir_bringup_sim robot.launch

	roslaunch mir_moveit_youbot_brsu_1 move_group.launch

	rosrun moveit_commander moveit_commander_cmdline.py

	rviz

### Launch Rockin Control fbm

	roslaunch mir_rockin_control_fbm_launcher rockin_control_fbm.launch

### Pre-assumption:
    move arm to:
    	c=[1.0 1.78146637708 -1.68232357583 3.40582651518 1.57]

    	or
    	[0.753206907905 1.76971351549 -1.67421755695 3.40553068428 1.00774327726]


## data publish

### calibrate origin

rostopic pub /compute_transform/reference_point geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'task_link'
point:
  x: 0.0
  y: 0.0
  z: 0.0"

	rostopic pub /compute_transform/event_in std_msgs/String "data: 'e_start'"


rostopic pub /path_generator/start_point geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'task_link'
point:
  x: 0.0
  y: 0.0
  z: 0.0"

rostopic pub /path_generator/end_point geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'task_link'
point:
  x: 0.1
  y: 0.1
  z: 0.0"

### send the pattern through csv file
* Line
	rostopic pub /pattern_selector std_msgs/String "/home/enigma/kinetic/src/sdp_ss2018_g5/mir_path_generator/ros/csv/pattern_line.csv"

* Sine wave
	rostopic pub /pattern_selector std_msgs/String "/home/enigma/kinetic/src/sdp_ss2018_g5/mir_path_generator/ros/csv/pattern_sine.csv"

### select the execution mode
* Normal mode
	rostopic pub /linear_interpolator_demo/mode std_msgs/String "data: 'normal'"

* Continuous mode
	rostopic pub /linear_interpolator_demo/mode std_msgs/String "data: 'continuous'"

### start pipeline

	rostopic pub /linear_interpolator_demo/event_in std_msgs/String "data: 'e_start'"

### execute motion

	rostopic pub /linear_interpolator_demo_trajectory_executor/event_in std_msgs/String "data: 'e_start'"

## debug topics:

	rostopic echo /linear_interpolator_demo/event_out

	rostopic echo /linear_interpolator_demo_ik_trajectory_solver/event_in

	rostopic echo /linear_interpolator_demo_ik_trajectory_solver/event_in

	rostopic echo /linear_interpolator_demo_ik_trajectory_solver/event_out

	rostopic echo /linear_interpolator_demo_ik_trajectory_solver/trajectory

	rostopic echo /mcr_manipulation/trajectory_time_parameterizer/trajectory_out
