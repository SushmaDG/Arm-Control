#!/usr/bin/env python
"""
This component provides functionality to generate
list of poses or a path given a start point and goal point
with in the domain of the selected path.

"""
#-*- encoding: utf-8 -*-
__author__ = 'youssef'

import rospy
import std_msgs.msg
import geometry_msgs.msg
import math
import tf
import numpy as np

class PathGenerator(object):
    """
    Generated a list of points(paths) given the start and end
    point with in the domain of the selected path.

    """
    def __init__(self):
        # params
        self.monitor_event = None
        self.start_point = None
        self.end_point = None

        self.read_parameters()

        # publishers
        self.path_pub = rospy.Publisher(
            '~path', geometry_msgs.msg.PoseArray
        )

        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String)

        # subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)

        rospy.Subscriber('~start_point', geometry_msgs.msg.PointStamped, self.start_point_cb)
        rospy.Subscriber('~end_point', geometry_msgs.msg.PointStamped, self.end_point_cb)

    def read_parameters(self):
        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        self.path_selector = rospy.get_param('~path_selector', "line")

        self.step_size = rospy.get_param('~step_size', 0.01)

        self.line_slope = rospy.get_param('~line_slope', 1.0)

        self.sine_amplitude = rospy.get_param('~sine_amplitude', 0.1)

        self.sine_angle_conversion_factor = rospy.get_param('~sine_angle_conversion_factor', 39.26990)

    def start(self):
        """
        Starts the component-wise pose error calculator.

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                state = self.init_state()
            elif state == 'IDLE':
                state = self.idle_state()
            elif state == 'RUNNING':
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def event_in_cb(self, msg):
        print "msg:.................", msg
        """
        Obtains an event for the component-wise pose error calculator.

        """
        self.monitor_event = msg.data

    def start_point_cb(self, msg):
        """
        Obtains the first pose.

        """
        self.start_point = msg

    def end_point_cb(self, msg):
        """
        Obtains the first pose.

        """
        self.end_point = msg

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.monitor_event == 'e_stop':
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'

        if self.monitor_event == 'e_start':
            return 'IDLE'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.monitor_event == 'e_stop':
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'

        if self.start_point and self.end_point:
            return 'RUNNING'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.monitor_event == 'e_stop':
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        else:
            computed_path = self.generate_path(self.start_point, self.end_point)

            if computed_path:
                self.path_pub.publish(computed_path)
                self.event_out.publish('e_success')
            else:
                self.event_out.publish('e_failure')

            self.reset_component_data()
            return 'INIT'

    def generate_path(self, start_point, end_point):
        """
        Generates path based on the specified parameters
        and the given start and goal points.

        :return: The generated path.
        :rtype: geometry_msgs.msg.PoseArray()

        """
        self.read_parameters()

        poseArray = geometry_msgs.msg.PoseArray()
        poseArray.header.stamp = rospy.Time.now()
        poseArray.header.frame_id = self.end_point.header.frame_id

        x_start = start_point.point.x
        y_start = start_point.point.y
        z_start = start_point.point.z

        x_end = end_point.point.x
        y_end = end_point.point.y
        z_end = end_point.point.z

        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0).tolist()

        no_of_points = 0.0

        if (self.step_size <= 0.0):
            rospy.logwarn('step size must be greater then zero.')
            return None

        no_of_points = (math.fabs(x_end - x_start)/ self.step_size)

        if no_of_points < 2:
            rospy.logwarn('Step size is larger than the distance between start and goal state.')
            return None

        X = np.linspace(x_start+self.step_size , x_end, no_of_points+1, endpoint=True)

        Y = self.generate_points(X)

        if Y is None:
            return None

        Z = np.linspace(z_start , z_end, no_of_points+1, endpoint=True)

        X_list = X.tolist()
        Y_list = Y.tolist()
        Z_list = Z.tolist()

        for index, val in enumerate(X_list):

            pose_temp = geometry_msgs.msg.Pose()
            pose_temp.position.x = val
            pose_temp.position.y = Y_list[index]
            pose_temp.position.z = Z_list[index]

            pose_temp.orientation.x = quat[0]
            pose_temp.orientation.y = quat[1]
            pose_temp.orientation.z = quat[2]
            pose_temp.orientation.w = quat[3]

            poseArray.poses.append(pose_temp)

        if len(poseArray.poses) == 0:
            rospy.logdebug('Length of generated path is zero.')
            return None

        return poseArray

    def generate_points(self, x):
        """
        Generates output coordinates given a list of input x-coordinates
        using the mathematical description of the selected path.

        :return: The generated output coordinates.
        :rtype: geometry_msgs.msg.PoseArray()

        """
        y = None
        if self.path_selector == "line":
            y = self.line_slope * x
        elif self.path_selector == "sine":
            y = self.sine_amplitude*np.sin(self.sine_angle_conversion_factor*x)
        else:
            rospy.logdebug('Invalid path selection.')

        return y

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.monitor_event = None
        self.start_point = None
        self.end_point = None

def main():
    rospy.init_node('compute_transform_node', anonymous=True)
    path_generator = PathGenerator()
    path_generator.start()
