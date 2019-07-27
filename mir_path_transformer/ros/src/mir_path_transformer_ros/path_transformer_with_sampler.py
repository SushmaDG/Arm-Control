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
import mcr_manipulation_utils_ros.kinematics as kinematics
import actionlib
import moveit_msgs
import moveit_commander

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
        #rospy.Subscriber("/all_parts_done", std_msgs.msg.String, self.all_done_cb)

        self.ik_solver_init()

    def ik_solver_init(self):
        # Params
        self.poses = None

        # Reference frame for the trajectory
        self.reference_frame = 'base_link'

        # Joint names of the arm
        self.joint_names = ['arm_joint_1','arm_joint_2', 'arm_joint_3',
                            'arm_joint_4','arm_joint_5']

        # Move group for MoveIt!
        move_group = 'move_group'

        client = actionlib.SimpleActionClient(move_group, moveit_msgs.msg.MoveGroupAction)
        rospy.loginfo("Waiting for '{0}' server".format(move_group))
        client.wait_for_server()
        rospy.loginfo("Found server '{0}'".format(move_group))

        # Name of the group to compute the inverse kinematics
        self.arm = 'arm_1'
        self.group = moveit_commander.MoveGroupCommander(self.arm)

        # Kinematics class to compute the inverse kinematics
        self.kinematics = kinematics.Kinematics(self.arm)

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

    # def all_done_cb(self, msg):
    #     rospy.loginfo("Transformer: All Done Here")
    #     if msg.data == "all_done":
    #         self.event_out.publish('e_success')

    def event_in_cb(self, msg):
        """
        Obtains an event for the component-wise pose error calculator.

        """
        self.monitor_event = msg.data

    def path_cb(self, msg):
        """
        Obtains the first pose.

        """
        rospy.loginfo("Transformer Path received")
        self.path = msg

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.monitor_event == 'e_start':
            rospy.loginfo("In path_transformer: Heard e_start")
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
                self.path = None
                self.transformed_path_pub.publish(transformed_path)
                self.event_out.publish('e_success')
                return 'INIT'
                #self.event_out.publish('e_success')
            else:
                self.event_out.publish('e_failure')

            self.reset_component_data()
            return 'INIT'

    def compute_transformed_path(self, path):

        transformed_path_wrt_wrist = self.transform_path(path, self.wrist_frame)


        transformed_path_wrt_wrist = self.prepare_top_grasp_path(
                                                                transformed_path_wrt_wrist,
                                                                self.tool_tip_frame,'arm_link_5'
                                                                )

        feasiable_path = self.find_feasiable_path(transformed_path_wrt_wrist)

        return feasiable_path

    def prepare_top_grasp_path(self, path, target_frame, source_frame):

        wrist_wrt_tooltip_pose = self.get_transformation(target_frame, source_frame)

        transformed_path_orientation = geometry_msgs.msg.PoseArray()
        transformed_path_orientation.header =  path.header

        for idx, pose in enumerate(path.poses):
            pose_temp = geometry_msgs.msg.PoseStamped()
            pose_temp.header = path.header
            pose_temp.pose = pose

            pose_temp.pose.orientation = wrist_wrt_tooltip_pose.orientation

            transformed_path_orientation.poses.append(pose_temp.pose)

        return transformed_path_orientation

    def find_feasiable_path(self, path):

        feasiable_path = geometry_msgs.msg.PoseArray()

        reference_pose = path.poses[0]
        for pose in path.poses:

            feasiable_pose = self.find_feasiable_pose(pose, reference_pose,
                                     path.header.frame_id
                                     )

            if feasiable_pose:
                reference_pose = feasiable_pose.pose

                feasiable_pose.header.frame_id = self.target_frame

                self.broadcast_transform(feasiable_pose)
                self.frame_id = self.frame_id + 1

                feasiable_path.poses.append(feasiable_pose.pose)
            else:
                return None

        feasiable_path.header.frame_id = self.target_frame

        return feasiable_path

    def find_feasiable_pose1(self, pose, reference_pose, target_frame):

        wrist_wrt_tooltip_pose = self.get_transformation(self.tool_tip_frame,
                                                         self.wrist_frame)

        sampler_direction = self.determine_sample_direction(reference_pose, pose)

        pose_in_wrist = self.calculate_transformation(pose, wrist_wrt_tooltip_pose)

        sampled_poses = self.pose_sampler(pose_in_wrist.pose, sampler_direction)

        transformed_sampled_poses_array = geometry_msgs.msg.PoseArray()
        transformed_sampled_poses_array.header.frame_id =  target_frame

        transformed_sampled_poses_array.poses = sampled_poses

        transformed_sampled_poses_wrt_target = self.transform_path(
                                                        transformed_sampled_poses_array,
                                                        self.target_frame
                                                        )

        return self.compute_ik_solutions(transformed_sampled_poses_wrt_target)


    def find_feasiable_pose(self, pose, reference_pose, target_frame):

        wrist_wrt_tooltip_pose = self.get_transformation(self.tool_tip_frame,
                                                         self.wrist_frame)

        sampler_direction = self.determine_sample_direction(reference_pose, pose)

        sampled_poses = self.pose_sampler(pose, sampler_direction)

        transformed_sampled_poses = self.transform_sampled_poses(sampled_poses)

        transformed_sampled_poses_array = geometry_msgs.msg.PoseArray()
        transformed_sampled_poses_array.header.frame_id =  target_frame
        transformed_sampled_poses_array.poses = transformed_sampled_poses

        transformed_sampled_poses_wrt_target = self.transform_path(
                                                        transformed_sampled_poses_array,
                                                        self.target_frame
                                                        )

        return self.compute_ik_solutions(transformed_sampled_poses_wrt_target)

    def transform_sampled_poses(self, poses):

        wrist_wrt_tooltip_pose = self.get_transformation(self.tool_tip_frame,
                                                            self.wrist_frame)

        transformed_sampled_poses = []
        for idx, pose in enumerate(poses):
            pose_in_wrist = self.calculate_transformation(pose, wrist_wrt_tooltip_pose)
            #self.broadcast_transform(pose_in_wrist)
            transformed_sampled_poses.append(pose_in_wrist.pose)
            #self.frame_id = self.frame_id + 1

        return transformed_sampled_poses

    def compute_ik_solutions(self, poses):
        """
        Given a list of poses, it returns a list of joint configurations (as a list)
        that are the solutions for each pose.

        :param poses: A list of poses
        :type poses: geometry_msgs.msg.PoseArray

        :return: A list of joint configurations that reach each pose.
        :rtype: list or None

        """
        # IK solver requires a PoseStamped object
        temp_poses = [geometry_msgs.msg.PoseStamped(pose=pose) for pose in poses.poses]
        rospy.logdebug("Calculating IK solution for {0} poses...".format(len(temp_poses)))

        for pose in temp_poses:
            solution = None
            solution = self.kinematics.inverse_kinematics(pose)
            if solution is not None:
                return pose

        rospy.logwarn("IK solution not found for any given pose")
        return None

    def determine_sample_direction(self, reference_pose, target_pose):
        p1 = KDL.Vector(reference_pose.position.x,
                        reference_pose.position.y,
                        reference_pose.position.z)

        p2 = KDL.Vector(target_pose.position.x,
                        target_pose.position.y,
                        target_pose.position.z)

        p1.Normalize()
        p2.Normalize()

        p1_x_p2 = p1*p2

        #print p1_x_p2

        if p1_x_p2.y() <= 0.0:
            return -1

        return 1

    def pose_sampler(self, pose, sampler_direction):
        angular_step_size = sampler_direction * 0.1

        sampled_angle_limit = sampler_direction * math.pi/4

        angle = 0.0

        quat1 = [pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,]

        sampled_poses = []
        while math.fabs(angle) <= math.fabs(sampled_angle_limit):

            quat2 = tf.transformations.quaternion_from_euler(0, angle, 0).tolist()

            quat1_x_quat2 = tf.transformations.quaternion_multiply(quat1, quat2).tolist()

            pose_temp = geometry_msgs.msg.Pose()
            pose_temp.position = pose.position
            pose_temp.orientation.x = quat1_x_quat2[0]
            pose_temp.orientation.y = quat1_x_quat2[1]
            pose_temp.orientation.z = quat1_x_quat2[2]
            pose_temp.orientation.w = quat1_x_quat2[3]

            sampled_poses.append(pose_temp)

            angle = angle + angular_step_size

        return sampled_poses

    def transform_path(self, path, target_frame):
        transformed_path = geometry_msgs.msg.PoseArray()

        for idx, pose in enumerate(path.poses):
            pose_temp = geometry_msgs.msg.PoseStamped()
            pose_temp.pose = pose
            pose_temp.header.frame_id = path.header.frame_id

            transformed_pose = self.transform_pose(pose_temp, target_frame)
            #self.broadcast_transform(transformed_pose)

            #self.frame_id = self.frame_id+1

            if transformed_pose:
                transformed_path.poses.append(transformed_pose.pose)
            else:
                return None

        transformed_path.header.frame_id = target_frame

        return transformed_path

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
