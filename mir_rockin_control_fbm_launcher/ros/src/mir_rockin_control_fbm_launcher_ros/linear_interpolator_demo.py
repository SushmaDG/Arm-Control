#!/usr/bin/env python
"""
A demo to generate a trajectory using linear interpolation and subsequently moving
the arm to follow such trajectory.

This component serves as a configurator/coordinator, i.e. it sets the required
parameters for all the components and starts/stops them accordingly.

**Input(s):**
  * `start_pose`: The pose where the manipulator's end effector will start the
  trajectory.
  * `goal_pose`: The pose where the manipulator's end effector will end the trajectory.

**Output(s):**
  * `trajectory`: The trajectory, in joint space, connecting the start and goal poses as
  a goal.

"""
#-*- encoding: utf-8 -*-

import rospy
import std_msgs.msg


class LinearInterpolationDemo(object):
    """
    Coordinates components that compute and execute a joint trajectory based on a
    start and goal pose.

    """
    def __init__(self):
        # Params
        self.started_components = False
        self.event = None
        self.linear_interpolator_status = None
        self.ik_solver_status = None
        self.trajectory_generator_status = None
        self.linear_transformer_status = None
        self.msg_data = None

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String)
        self.start_linear_interpolator = rospy.Publisher(
            '~start_linear_interpolator', std_msgs.msg.String, latch=True
        )
        self.start_linear_transformer = rospy.Publisher(
            '~start_linear_transformer', std_msgs.msg.String, latch=True
        )
        self.start_ik_solver = rospy.Publisher(
            '~start_ik_solver', std_msgs.msg.String, latch=True
        )
        self.start_trajectory_generator = rospy.Publisher(
            '~start_trajectory_generator', std_msgs.msg.String, latch=True
        )
        self.part_complete = rospy.Publisher("/part_execution", std_msgs.msg.String, queue_size=1)

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            "~linear_interpolator_status", std_msgs.msg.String,
            self.linear_interpolator_status_cb
        )
        rospy.Subscriber(
            "~linear_transformer_status", std_msgs.msg.String,
            self.linear_transformer_status_cb
        )
        rospy.Subscriber(
            "~ik_solver_status", std_msgs.msg.String, self.ik_solver_status_cb
        )
        rospy.Subscriber(
            "~trajectory_generator_status", std_msgs.msg.String,
            self.trajectory_generator_status_cb
        )

        #gets the mode that it has to operate in - Normal or Continuous
        #rospy.Subscriber('/mode', std_msgs.msg.String, self.mode_in_cb)

        rospy.Subscriber("/all_parts_done", std_msgs.msg.String, self.all_done_cb)

    # def mode_in_cb(self, msg):
    #     """
    #     Gets the mode
    #     """
    #     self.mode = msg.data

    def all_done_cb(self, msg):
        rospy.loginfo("***********All Done Here**************")
        self.msg_data = msg.data

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def linear_interpolator_status_cb(self, msg):
        """
        Obtains the status of the trajectory generator (as an event).

        """
        self.linear_interpolator_status = msg.data

    def linear_transformer_status_cb(self, msg):
        """
        Obtains the status of the trajectory generator (as an event).

        """
        self.linear_transformer_status = msg.data

    def ik_solver_status_cb(self, msg):
        """
        Obtains the status of the ik solver (as an event).

        """
        self.ik_solver_status = msg.data

    def trajectory_generator_status_cb(self, msg):
        """
        Obtains the status of the joint trajectory generator (as an event).

        """
        self.trajectory_generator_status = msg.data

    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                state = self.init_state()
            elif state == 'RUNNING':
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_start':
            rospy.loginfo("Started coordinator: In init state")
            return 'RUNNING'
        else:
            return 'INIT'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        self.toggle_components(self.event)

        if self.event == 'e_stop':
            rospy.loginfo("Stopping Co-ordinator")
            status = 'e_stopped'
            self.event_out.publish(status)
            self.reset_component_data(status)
            return 'INIT'
        if (self.linear_interpolator_status == 'e_failure'
            or self.linear_transformer_status == 'e_failure'
            or self.ik_solver_status == 'e_failure'
                or self.trajectory_generator_status == 'e_failure'):
            status = 'e_failure'

            rospy.loginfo("Path generator status------")
            rospy.loginfo(self.linear_interpolator_status)
            rospy.loginfo("Transformer_status ------")
            rospy.loginfo(self.linear_transformer_status)
            rospy.loginfo("ik_solver_status------")
            rospy.loginfo(self.ik_solver_status)
            rospy.loginfo("trajectory_generator_status ------")
            rospy.loginfo(self.trajectory_generator_status)
            #rospy.loginfo("Stopping Co-ordinator Failure")

            self.event_out.publish(status)
            self.reset_component_data(status)
            return 'INIT'
        if self.trajectory_generator_status == 'e_success':
            status = 'e_success'
            rospy.loginfo("Exiting continuous mode: In coordinator")
            self.event_out.publish(status)
            self.reset_component_data(status)
            return 'INIT'
        if self.ik_solver_status == 'e_success':
            rospy.loginfo("Received success from IK")
            status = "e_start_new"
            self.part_complete.publish(status)
            self.ik_solver_status = None
            if self.msg_data == 'all_done':
                status = 'e_success'
                self.msg_data = None
                self.event_out.publish(status)
                self.reset_component_data(status)
                rospy.loginfo("In coordinator: sent e_stopped to toggle_components")
                return 'INIT'
            else:
                return 'RUNNING'
        else:
            return 'RUNNING'

    def toggle_components(self, event):
        """
        Starts or stops the necessary components based on the event.

        :param event: The event that determines either to start or stop the components.
        :type event: str

        """
        #if self.mode == 'normal':
        # rospy.loginfo("In toggle_components: Received event is")
        # rospy.loginfo(self.event)
        if event == 'e_stopped' or event == 'e_failure' or event == 'e_success':
            self.start_linear_interpolator.publish('e_stop')
            self.start_linear_transformer.publish('e_stop')
            self.start_ik_solver.publish('e_stop')
            self.start_trajectory_generator.publish('e_stop')
            self.started_components = False

        if event == 'e_start' and not self.started_components:
            rospy.loginfo("starting all components")
            self.start_linear_interpolator.publish('e_start')
            self.start_linear_transformer.publish('e_start')
            self.start_ik_solver.publish('e_start')
            self.start_trajectory_generator.publish('e_start')
            self.started_components = True

    def reset_component_data(self, result):
        """
        Clears the data of the component.

        :param result: The result of the component, e.g. stopped, failure, success.
        :type result: str

        """
        self.toggle_components(result)
        self.event = None
        self.linear_interpolator_status = None
        self.linear_transformer_status = None
        self.ik_solver_status = None
        self.trajectory_generator_status = None
        self.started_components = False


def main():
    rospy.init_node("linear_interpolator_demo", anonymous=True)
    linear_interpolator_demo = LinearInterpolationDemo()
    linear_interpolator_demo.start()
