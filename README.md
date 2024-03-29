## Arm Control

A common task in robot manipulation is to move the end-effector of a manipulator in a specified path (for example, to weld or paint a part). The code in the package mir rockin_control_fbm was developed for a competition 3 years ago; the task was to either follow (with the end-effector) a straight line of a given slope, or a sine wave path with a given frequency and amplitude.

### Problem

The code does not work (at least in simulation). It is also not easily transferable to other (similar tasks). For example, a new task is to follow a path which is not pre-specified, but instead on which is drawn on a sheet of paper. Hence the task would be to perceive and follow the path simultaneously. The existing code works only if the path is fully known beforehand.

### Proposed solution
* The non-working code has been narrowed down to mcr trajectory time parameterizer. Debug and make the component run.
* Refactor the code such that arm trajectories can be generated and executed as new information (about the path) is available.

### Setup
`git clone https://github.com/b-it-bots/mas_industrial_robotics.git`
  
### Launch 

roscore
roslaunch mir_bringup_sim robot.launch
rosrun gazebo_ros gzclient (only if you want to view the robot in gazebo)
roslaunch mir_moveit_youbot_brsu_1 move_group.launch
roslaunch mir_rockin_control_fbm_launcher rockin_control_fbm.launch
rviz

Minimal rviz configuration:
* set the global frame to odom
* add RobotModel
* add TF
Add other topics as necessary.

### Run

Send the arm to a preconfigured pose:

`rosrun moveit_commander moveit_commander_cmdline.py`

`use arm_1`

`c=[1.0 1.78146637708 -1.68232357583 3.40582651518 1.57]`

`go c`

This arm configuration can be saved in mir moveit youbot brsu 1/config/youbot.srdf, so you don’t have to type it in every time. The following commands are also in the README.md of the package mir rockin control fbm launcher.

`rostopic pub /compute_transform/reference_point geometry_msgs/PointStamped "header:`

`seq : 0`

`stamp :`

`secs : 0`

`nsecs : 0`

`rame id : ’ task link ’`

`point :`

`x : 0.0`

`y : 0.0`

`z : 0.0” `

`rostopic pub /compute_transform/event_in std_msgs/String "data: 'e_start' "`

`rostopic pub /path_generator/start_point geometry_msgs/PontStamped "header: `

`seq : 0`

`stamp :`

`secs : 0`

`nsecs : 0`

`frame id : ’ task link ’`

`point :`

`x : 0.0`

`y : 0.0`

`z : 0.0” `


`rostopic pub/path_generator/end_point geometry_msgs/PoseStamped "header: `

`seq : 0`

`stamp :`

`secs : 0`

`nsecs : 0`

`frame id : ’ task link ’`

`point :`

`x : 0.1`

`y : 0.1`

`z : 0.0” `


`rostopic linear_interpolator_demo/event_in std_msgs/String "data: 'e_start' " `

`rostopic linear_interpolator_demo_trajectory_executor/event_in std_msgs/String "data: 'e_start' " `


