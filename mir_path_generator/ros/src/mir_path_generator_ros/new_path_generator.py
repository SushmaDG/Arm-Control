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
import time


import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

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
        self.x = []
        self.y = []
        self.z = []
        self.pattern = None
        self.part_completion = None
        rospy.Subscriber('/pattern_selector', std_msgs.msg.String, self.pattern_in_cb)
        rospy.Subscriber('/part_execution', std_msgs.msg.String, self.part_execution_cb)

        # publishers
        self.path_pub = rospy.Publisher(
            '~path', geometry_msgs.msg.PoseArray
        )

        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String)
        self.all_done = rospy.Publisher('/all_parts_done', std_msgs.msg.String)

        # subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)

        rospy.Subscriber('~start_point', geometry_msgs.msg.PointStamped, self.start_point_cb)
        rospy.Subscriber('~end_point', geometry_msgs.msg.PointStamped, self.end_point_cb)


    def getPoints(self):
        self.x = []
        self.y = []
        self.z = []
        with open(self.pattern) as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                # print(row['X'], row['Y'],row['Z'])
                self.x.append(float(row['X']))
                self.y.append(float(row['Y']))
                self.z.append(float(row['Z']))
        # print(x,y,z)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(self.x, self.y, self.z)
        plt.show()
        self.number_of_points = len(self.x)
        self.window_size = 3
        self.number_of_windows = self.number_of_points/self.window_size
        self.index_increment = 0

    def partition_path(self, start_point, end_point):
        poseArray = geometry_msgs.msg.PoseArray()
        poseArray.header.stamp = rospy.Time.now()
        poseArray.header.frame_id = self.end_point.header.frame_id
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0).tolist()
        max_windowed_points = self.number_of_windows*self.window_size
        if self.index_increment < (max_windowed_points):
            range_list = range(self.index_increment,(self.index_increment+self.window_size))
        else:
            range_list = range(max_windowed_points,(self.number_of_points-max_windowed_points))
        rospy.loginfo("Range list:")
        rospy.loginfo(range_list)
        rospy.loginfo(len(poseArray.poses))
        for index in range_list:
            pose_temp = geometry_msgs.msg.Pose()
            pose_temp.position.x = self.x[index]
            pose_temp.position.y = self.y[index]
            pose_temp.position.z = self.z[index]

            pose_temp.orientation.x = quat[0]
            pose_temp.orientation.y = quat[1]
            pose_temp.orientation.z = quat[2]
            pose_temp.orientation.w = quat[3]

            poseArray.poses.append(pose_temp)

        if len(poseArray.poses) == 0:
            rospy.loginfo('Length of generated path is zero in partition_path')
            return None
        return poseArray


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
            elif state == 'WAITING':
                state = self.wait_for_completion_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def event_in_cb(self, msg):
        """
        Obtains an event for the component-wise pose error calculator.

        """
        self.monitor_event = msg.data

    def pattern_in_cb(self, msg):
        """
        Obtains an event for the component-wise pose error calculator.

        """
        rospy.loginfo("Pattern Selection Callback Called")
        if self.monitor_event != "e_start":
            self.pattern = msg.data
            self.getPoints()
            rospy.loginfo("Updating pattern")
        else:
            rospy.loginfo("System Busy : Cannot Update Pattern")

    def part_execution_cb(self, msg):
        """
        Obtains an event for the component-wise pose error calculator.

        """
        rospy.loginfo("Part Execution Callback Called--recieved message:")
        rospy.loginfo(msg.data)
        self.part_completion = msg.data
        if self.part_completion == "e_start_new":
            rospy.loginfo("Starting with new set of points")
            rospy.loginfo("Part Exection done")
            self.index_increment = self.index_increment + self.window_size


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
        if (self.monitor_event == 'e_stop'):
            rospy.loginfo("In path_generator: Received e_stop")
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
        if self.monitor_event == 'e_stop' or self.monitor_event == 'e_stopped':
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'

        #if self.start_point and self.end_point:
        if self.pattern != None:
            return 'RUNNING'
        else:
            rospy.logerr('Path Generator started without specifying the pattern')
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        rospy.loginfo('In Running STATE')
        if self.monitor_event == 'e_stop':
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        else:
            #computed_path = self.generate_path(self.start_point, self.end_point)
            if self.index_increment >= (self.number_of_windows*self.window_size):
                self.all_done.publish('all_done')
                rospy.loginfo("Publishing all done message to coordinator")
                self.event_out.publish('e_success')
                self.reset_component_data()
                return 'INIT'
            computed_path = self.partition_path(self.start_point, self.end_point)
            rospy.loginfo("Computed path: in path generator")
            rospy.loginfo(computed_path)
            if computed_path:
                self.path_pub.publish(computed_path)
                # self.event_out.publish('e_success')
                return 'WAITING'
            else:
                self.event_out.publish('e_failure')

            self.reset_component_data()
            return 'INIT'

    def wait_for_completion_state(self):
        rospy.loginfo('In Wait STATE')
        if self.monitor_event == 'e_stop' or self.monitor_event == 'e_stopped':
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        else:
            time.sleep(5)
            if self.part_completion == 'e_start_new':
                self.part_completion = None
                return 'RUNNING'
            else:
                rospy.loginfo("Wait for completion state: In else loop")
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
        self.index_increment = 0
        self.part_completion = None

def main():
    rospy.init_node('compute_transform_node', anonymous=True)
    path_generator = PathGenerator()
    path_generator.start()
