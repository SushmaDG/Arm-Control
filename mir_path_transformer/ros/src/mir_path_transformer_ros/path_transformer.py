#!/usr/bin/env python
"""
This module contains a component that calculates
the component-wise error between two poses.

"""
#-*- encoding: utf-8 -*-
__author__ = 'jsanch'

import rospy
import std_msgs.msg
import geometry_msgs.msg
import tf
import math
import numpy
import PyKDL as KDL

class PathTransformer(object):
    """
    Calculates the error between two poses in three
    linear components and three angular components.

    """
    def __init__(self):
        # params
        self.monitor_event = None

        self.path = None

        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.frame_id = 0

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # how long to wait for transform (in seconds)
        self.wait_for_transform = rospy.get_param('~wait_for_transform', 0.1)

        self.reference_frame = rospy.get_param('~reference_frame', 'arm_link_5')

        self.target_frame = rospy.get_param('~target_frame', 'base_link')

        self.tool_tip_frame = rospy.get_param('~tool_tip_frame', 'gripper_tip_link')

        self.wrist_frame = rospy.get_param('~wrist_frame', 'arm_link_5')

        # publishers
        self.transformed_path_pub = rospy.Publisher(
            '~transformed_path', geometry_msgs.msg.PoseArray
        )
        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String)

        # subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber('~path', geometry_msgs.msg.PoseArray, self.path_cb)

    def start(self):
        """
        Starts the path transformer.

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
        """
        Obtains an event for the component-wise pose error calculator.

        """
        self.monitor_event = msg.data

    def path_cb(self, msg):
        """
        Obtains the first pose.

        """
        self.path = msg

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
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

        if self.path:
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
            return 'INIT'
        else:
            transformed_path = self.compute_transformed_path(self.path)
            if transformed_path:
                self.transformed_path_pub.publish(transformed_path)
                self.event_out.publish('e_success')
            else:
                self.event_out.publish('e_failure')

            self.reset_component_data()
            return 'INIT'

    def compute_transformed_path(self, path):
        transformed_path = geometry_msgs.msg.PoseArray()

        for idx, pose in enumerate(path.poses):
            pose_temp = geometry_msgs.msg.PoseStamped()
            pose_temp.pose = pose
            pose_temp.header.frame_id = path.header.frame_id

            self.frame_id = idx

            transformed_pose = self.compute_transformed_pose(pose_temp)

            if transformed_pose:
                transformed_path.poses.append(transformed_pose.pose)
            else:
                return None

        transformed_path.header.frame_id = self.target_frame

        return transformed_path

    def compute_transformed_pose(self, pose):

        tranformed_pose = self.transform_pose(pose, self.wrist_frame)

        if tranformed_pose:
            wrist_wrt_tooltip_pose = self.get_transformation(self.tool_tip_frame, self.wrist_frame)
            tranformed_pose.pose.orientation = wrist_wrt_tooltip_pose.orientation

            if wrist_wrt_tooltip_pose:
                #project poses into wrist frame
                deisred_wrist_pose = self.calculate_transformation(tranformed_pose.pose, wrist_wrt_tooltip_pose)

                # transform pose into target frame
                pose_wrt_target_frame = self.transform_pose(deisred_wrist_pose, self.target_frame)

                self.broadcast_transform(pose_wrt_target_frame)
                return pose_wrt_target_frame

            return None

        else:
            return None

    def transform_pose(self, pose, target_frame):
        """
        Gets transformation of the reference frame into the target frame.

        :return: The eef pose into target frame.
        :rtype: geometry_msgs.msg.PoseStamped or None

        """

        try:
            common_time = self.listener.getLatestCommonTime(
                target_frame, pose.header.frame_id
            )

            self.listener.waitForTransform(
                target_frame, pose.header.frame_id,
                common_time, rospy.Duration(self.wait_for_transform)
            )

            pose.header.stamp = common_time

            trasformed_pose = self.listener.transformPose(target_frame, pose)

            if trasformed_pose:

                return trasformed_pose

            else:
                return None

        except tf.Exception, error:
            rospy.logwarn("Exception occurred: {0}".format(error))
            return None

        return None

    def calculate_transformation(self, pose_1 , pose_2):

        pose_1_matrix = self.pose_to_matrix(pose_1)

        pose_2_matrix = self.pose_to_matrix(pose_2)

        return  self.matrix_to_pose(self.wrist_frame,
                    numpy.dot(pose_1_matrix, pose_2_matrix))

    def get_transformation(self, target_frame, source_frame):
        """
        Gets transformation of the reference frame into the target frame.

        :return: The eef pose into target frame.
        :rtype: geometry_msgs.msg.PoseStamped or None

        """

        pose = geometry_msgs.msg.Pose()
        try:
            common_time = self.listener.getLatestCommonTime(
                target_frame, source_frame
            )

            self.listener.waitForTransform(
                target_frame, source_frame,
                common_time, rospy.Duration(self.wait_for_transform)
            )

            (trans,quat) = self.listener.lookupTransform(
                target_frame, source_frame, common_time)

            pose.position.x = trans[0]
            pose.position.y = trans[1]
            pose.position.z = trans[2]

            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            return pose

        except tf.Exception, error:
            rospy.logwarn("Exception occurred: {0}".format(error))
            return None

        return None

    def pose_to_matrix(self, pose):

        quat = [pose.orientation.x, \
                pose.orientation.y, \
                pose.orientation.z, \
                pose.orientation.w]
        position = [[pose.position.x], \
                    [pose.position.y], \
                    [pose.position.z]]

        matrix_form = tf.transformations.quaternion_matrix(quat)
        matrix_form[0:3,3:4] = numpy.matrix(position)

        return matrix_form

    def matrix_to_pose(self,frame, matrix):
        '''
        :param frame: Name of the reference frame in which the pose is
        specified.
        :type frame: String

        :param matrix: The 4x4 transformation matrix.
        :type matrix: numpy.matrix

        :return: The pose interpretable by ROS.
        :rtype: geometry_msgs.msg.PoseStamepd
        '''
        pose = geometry_msgs.msg.PoseStamped()

        pose.header.frame_id = frame
        pose.header.stamp = rospy.Time.now()

        pose.pose.position.x = matrix[0, 3]
        pose.pose.position.y = matrix[1, 3]
        pose.pose.position.z = matrix[2, 3]

        quat = tf.transformations.quaternion_from_matrix(matrix)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        return pose

    def broadcast_transform(self, transformed_pose):
        pose = transformed_pose.pose
        pos  = pose.position
        quat = pose.orientation
        self.br.sendTransform((pos.x, pos.y, pos.z),
                     (quat.x, quat.y, quat.z, quat.w),
                     rospy.Time.now(),
                     "transformed_pose"+str(self.frame_id),
                     transformed_pose.header.frame_id
                     )

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.monitor_event = None
        self.frame_id = 0
        self.path = None

def main():
    rospy.init_node('path_transformer_node', anonymous=True)
    path_transformer = PathTransformer()
    path_transformer.start()
