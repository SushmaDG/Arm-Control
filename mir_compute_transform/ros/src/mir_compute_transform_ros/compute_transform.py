#!/usr/bin/env python
"""
This module contains a component that computes a transformation,
given reference point.

"""
#-*- encoding: utf-8 -*-
__author__ = 'jsanch'

import rospy
import std_msgs.msg
import geometry_msgs.msg
import tf
import math
import numpy

class ComputeTransform(object):
    """
    computes a transformation based on given reference point.

    """
    def __init__(self):
        # params
        self.monitor_event = None
        self.reference_point = None

        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        #self.theta = -math.pi/2

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # how long to wait for transform (in seconds)
        self.wait_for_transform = rospy.get_param('~wait_for_transform', 0.1)

        self.theta = rospy.get_param('~world_frame_rotation', 0.0)

        self.reference_frame = rospy.get_param('~reference_frame', 'gripper_tip_link')

        self.target_frame = rospy.get_param('~target_frame', 'base_link')

        # publishers
        self.transformed_pose_pub = rospy.Publisher(
            '~transformed_pose', geometry_msgs.msg.PoseStamped
        )
        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String)

        # subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber('~reference_point', geometry_msgs.msg.PointStamped, self.reference_point_cb)

        self.computed_transform = None

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
            elif state == 'PROCESSING':
                state = self.processing_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def event_in_cb(self, msg):
        """
        Obtains an event for the component-wise pose error calculator.

        """
        self.monitor_event = msg.data

    def reference_point_cb(self, msg):
        """
        Obtains the first pose.

        """
        self.reference_point = msg

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
        elif self.monitor_event == 'e_start':
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

        if self.reference_point:
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
            eef_pose = self.get_eef_transform()

            if eef_pose:
                self.computed_transform = self.compute_transform(
                    self.reference_point, eef_pose
                )
                self.event_out.publish('e_success')
            else:
                self.computed_transform = None
                self.event_out.publish('e_failure')

            return 'PROCESSING'

    def processing_state(self):
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
            if self.computed_transform:
                self.transformed_pose_pub.publish(self.computed_transform)
                self.broadcast_transform(self.computed_transform)
            else:
                return 'INIT'

            return 'PROCESSING'

    def broadcast_transform(self, transform_pose):
        pose = transform_pose.pose
        pos  = pose.position
        quat = pose.orientation
        self.br.sendTransform((pos.x, pos.y, pos.z),
                     (quat.x, quat.y, quat.z, quat.w),
                     rospy.Time.now(),
                     self.reference_point.header.frame_id,
                     transform_pose.header.frame_id
                     )

    def get_eef_transform(self):
        """
        Gets transformation of the reference frame into the target frame.

        :return: The eef pose into target frame.
        :rtype: geometry_msgs.msg.PoseStamped or None

        """

        pose = geometry_msgs.msg.Pose()
        try:
            common_time = self.listener.getLatestCommonTime(
                self.target_frame, self.reference_frame
            )

            self.listener.waitForTransform(
                self.target_frame, self.reference_frame,
                common_time, rospy.Duration(self.wait_for_transform)
            )

            (trans,quat) = self.listener.lookupTransform(
                self.target_frame, self.reference_frame, common_time)

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

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.monitor_event = None
        self.reference_point = None

    def compute_transform(self, reference_point, eef_pose):
        """
        Calculates the transform between the target frame and the frame of the input point.

        :param reference_point: The point in the world frame.
        :type reference_point: geometry_msgs.msg.Point

        :param eef_pose: The eef pose.
        :type eef_pose: geometry_msgs.msg.PoseStamped

        :return: Tramsform as pose stamped.
        :rtype: geometry_msgs.msg.PoseStamped

        """
        #==============base to eef============
        base_to_eef_quat = [eef_pose.orientation.x, \
                eef_pose.orientation.y, \
                eef_pose.orientation.z, \
                eef_pose.orientation.w]

        base_to_eef_position = [[eef_pose.position.x], \
                            [eef_pose.position.y], \
                            [eef_pose.position.z]]

        T_base_eef = tf.transformations.quaternion_matrix(base_to_eef_quat)

        T_base_eef[0:3,3:4] = numpy.matrix(base_to_eef_position)

        #===============station to goal=======================
        station_to_goal_quat = [0.0, 0.0, 0.0, 1.0]
        station_to_goal_position = [[reference_point.point.x], \
                            [reference_point.point.y], \
                            [reference_point.point.z]]

        T_station_goal = tf.transformations.quaternion_matrix(station_to_goal_quat)
        T_station_goal[0:3,3:4] = numpy.matrix(station_to_goal_position)

        R_station_goal = tf.transformations.quaternion_matrix(station_to_goal_quat)
        P_station_goal = numpy.matrix(station_to_goal_position)

        T_goal_station = tf.transformations.identity_matrix()

        #T_goal_station = tf.transformations.inverse_matrix(T_station_goal)
        
        T_goal_station[0:3,0:3] = T_station_goal[0:3,0:3].transpose()

        T_goal_station[0:3,3:4] = -(T_goal_station[0:3,0:3]*P_station_goal)

        #=============eef to goal translation============
        R_base_eef = T_base_eef[0:3,0:3]

        R_station_goal = T_station_goal[0:3,0:3]

        R_base_station = numpy.matrix([
                            [math.cos(self.theta), -math.sin(self.theta), 0.0,],
                            [math.sin(self.theta), math.cos(self.theta), 0.0,],
                            [0.0, 0.0, 1.0]])

        R_eef_goal = R_base_eef.transpose() * R_base_station * R_station_goal

        eef_to_goal_position = [[0.0], \
                            [0.0], \
                            [0.0]]

        T_eef_goal = tf.transformations.identity_matrix()

        T_eef_goal[0:3,0:3] = R_eef_goal

        T_eef_goal[0:3,3:4] = numpy.matrix(eef_to_goal_position)

        #=======================compute transformation===============
        temp = numpy.dot(T_base_eef, T_eef_goal)

        T_base_station = numpy.dot(temp, T_goal_station)

        return self.matrix_to_pose(self.target_frame, T_base_station)

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

def main():
    rospy.init_node('compute_transform_node', anonymous=True)
    compute_transform = ComputeTransform()
    compute_transform.start()
