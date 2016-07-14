#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, TORK (Tokyo Opensource Robotics Kyokai Association)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Tokyo Opensource Robotics Kyokai Association
#    nor the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Ryu Yamamoto, Isaac I. Y. Saito

import math

from geometry_msgs.msg import Pose
import rospy
from tf.transformations import quaternion_from_euler

from nextage_ros_bridge.nextage_client import NextageClient


class SampleMoveitCommander():
    '''
    Sample script that runs some tasks on NEXTAGE Open.
    Also intended to be used from test cases. 
    '''
    _KINEMATICSOLVER_SAFE = 'RRTConnectkConfigDefault'

    _MOVEGROUP_NAME_TORSO = 'torso'
    _JOINTNAMES_TORSO = ['CHEST_JOINT0']
    # Set of MoveGroup name and the list of joints
    _MOVEGROUP_ATTR_TORSO = [_MOVEGROUP_NAME_TORSO, _JOINTNAMES_TORSO]

    _MOVEGROUP_NAME_LEFTARM = 'left_arm'
    _JOINTNAMES_LEFTARM = ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5']
    _MOVEGROUP_ATTR_LEFTARM = [_MOVEGROUP_NAME_LEFTARM, _JOINTNAMES_LEFTARM]

    _MOVEGROUP_NAME_RIGHTARM = 'right_arm'
    _JOINTNAMES_RIGHTARM = ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5']
    _MOVEGROUP_ATTR_RIGHTARM = [_MOVEGROUP_NAME_RIGHTARM, _JOINTNAMES_RIGHTARM]

    _MOVEGROUP_NAME_LEFTHAND = 'left_hand'
    _JOINTNAMES_LEFTHAND = ['LARM_JOINT5']
    _MOVEGROUP_ATTR_LEFTHAND = [_MOVEGROUP_NAME_LEFTHAND, _JOINTNAMES_LEFTHAND]

    _MOVEGROUP_NAME_RIGHTHAND = 'right_hand'
    _JOINTNAMES_RIGHTHAND = ['RARM_JOINT5']
    _MOVEGROUP_ATTR_RIGHTHAND = [_MOVEGROUP_NAME_RIGHTHAND, _JOINTNAMES_RIGHTHAND]

    _MOVEGROUP_NAME_HEAD = 'head'
    _JOINTNAMES_HEAD = ['HEAD_JOINT0', 'HEAD_JOINT1']
    _MOVEGROUP_ATTR_HEAD = [_MOVEGROUP_NAME_HEAD, _JOINTNAMES_HEAD]

    _MOVEGROUP_NAME_BOTHARMS = 'botharms'
    _JOINTNAMES_BOTHARMS = _JOINTNAMES_TORSO + _JOINTNAMES_LEFTARM + _JOINTNAMES_RIGHTARM
    _MOVEGROUP_ATTR_BOTHARMS = [_MOVEGROUP_NAME_BOTHARMS, _JOINTNAMES_BOTHARMS]

    _MOVEGROUP_NAME_UPPERBODY = 'upperbody'
    _JOINTNAMES_UPPERBODY = _JOINTNAMES_TORSO + _JOINTNAMES_HEAD + _JOINTNAMES_LEFTARM + _JOINTNAMES_RIGHTARM
    _MOVEGROUP_ATTR_UPPERBODY = [_MOVEGROUP_NAME_UPPERBODY, _JOINTNAMES_UPPERBODY]

    # List of all MoveGroup set
    _MOVEGROUP_NAMES = sorted([_MOVEGROUP_NAME_TORSO, _MOVEGROUP_NAME_HEAD,
                               _MOVEGROUP_NAME_LEFTARM, _MOVEGROUP_NAME_RIGHTARM,
                               _MOVEGROUP_NAME_LEFTHAND, _MOVEGROUP_NAME_RIGHTHAND,
                               _MOVEGROUP_NAME_BOTHARMS, _MOVEGROUP_NAME_UPPERBODY])
    _MOVEGROUP_ATTRS = [_MOVEGROUP_ATTR_TORSO, _MOVEGROUP_ATTR_HEAD,
                        _MOVEGROUP_ATTR_LEFTARM, _MOVEGROUP_ATTR_RIGHTARM,
                        _MOVEGROUP_ATTR_LEFTHAND, _MOVEGROUP_ATTR_RIGHTHAND,
                        _MOVEGROUP_ATTR_BOTHARMS, _MOVEGROUP_ATTR_UPPERBODY]

    _TARGETPOSE_LEFT = []  # TODO fill this

    def __init__(self):
        self._robot = NextageClient()
        rospy.init_node("nxo_moveit_samplenode")

        # TODO: Read groups from SRDF file ideally.

        for mg_attr in self._MOVEGROUP_ATTRS:
            mg = MoveGroupCommander(mg_attr[0])
            # Temporary workaround of planner's issue similar to https://github.com/tork-a/rtmros_nextage/issues/170
            mg.set_planner_id(self._KINEMATICSOLVER_SAFE)
            # Append MoveGroup instance to the MoveGroup attribute list.
            mg_attr.append(mg)

    def _set_sample_pose(self):
        '''
        @return: Pose() with some values populated in.
        '''
        pose_target = Pose()
        pose_target.orientation.x = 0.00
        pose_target.orientation.y = 0.00
        pose_target.orientation.z = -0.20
        pose_target.orientation.w = 0.98
        pose_target.position.x = 0.18
        pose_target.position.y = -0.00
        pose_target.position.z = 0.94
        return pose_target

    def _plan(self):
        '''
        Run `clear_pose_targets` at the beginning.
        @rtype: RobotTrajectory http://docs.ros.org/api/moveit_msgs/html/msg/RobotTrajectory.html
        '''
        self._mvgroup.clear_pose_targets()

        pose_target = self._set_sample_pose()
        self._mvgroup.set_pose_target(pose_target)
        plan = self._mvgroup.plan()
        rospy.loginfo('  plan: '.format(plan))
        return plan

    def _plan_leftarm(self, movegroup):
        '''
        @type movegroup: MoveGroupCommander
        @rtype: RobotTrajectory http://docs.ros.org/api/moveit_msgs/html/msg/RobotTrajectory.html
        '''
        movegroup.clear_pose_targets()

        pose_target = Pose()
        pose_target.orientation.x = -0.32136357
        pose_target.orientation.y = -0.63049522
        pose_target.orientation.z = 0.3206799
        pose_target.orientation.w = 0.62957575
        pose_target.position.x = 0.32529
        pose_target.position.y = 0.29919
        pose_target.position.z = 0.24389

        movegroup.set_pose_target(pose_target)
        plan = movegroup.plan()  # TODO catch exception
        rospy.loginfo('Plan: '.format(plan))
        return plan

    def _armgroup_plango(self, armgroup, pose, ref_frame='None'):
        '''
        Check if http://wiki.ros.org/rtmros_nextage/Tutorials/Programming_Hiro_NEXTAGE_OPEN_MOVEIT works
        @param armgroup: instanceof MoveGroup.
        @return instance of MoveGroup where the goal pose is set.
        '''
        # Light Arm Initial Pose
        arm_initial_pose = armgroup.get_current_pose().pose
        print "=" * 10, " Printing Left Hand initial pose: "
        print arm_initial_pose
        armgroup.set_pose_target(pose, ref_frame)
        return armgroup.go()

    def sample_left_plango(self, pose):
        '''
        Check if http://wiki.ros.org/rtmros_nextage/Tutorials/Programming_Hiro_NEXTAGE_OPEN_MOVEIT works
        '''
        print("=" * 10, " Printing left arm initial pose: ")
        print(pose)
        return self._armgroup_plango(self.robot._MOVEGROUP_ATTR_LEFTARM[2], pose)

    def sample_left_plango_2(self):
        pose_l_target_arbitrary = Pose([ 0.325471850974 - 0.01, -0.182271241593 - 0.3, 0.0676272396419 + 0.3],
                                       [ -0.000556712307053, -0.706576742941, -0.00102461782513, 0.707635461636 ])
        return self.sample_left_plango(pose_l_target_arbitrary)

    def sample_right_plango(self, pose):
        '''
        Check if http://wiki.ros.org/rtmros_nextage/Tutorials/Programming_Hiro_NEXTAGE_OPEN_MOVEIT works
        '''
        print("=" * 10, " Printing right arm initial pose: ")
        print(pose)
        return self._armgroup_plango(self.robot._MOVEGROUP_ATTR_RIGHTARM[2], pose)

    def sample_right_plango_1(self):
        '''Right arm initial pose'''
        pose_r_init = Pose([ 0.325575143293, -0.182363208375, 0.0676247521028 ],
                           [ 0.000110040644185, -0.707732376826, 2.44788398956e-05, 0.706480622582 ])
        return self.sample_right_plango(pose_r_init)

    def sample_right_plango_2(self):
        '''Right arm arbitrary pose'''
        pose_r_target_arbitrary = Pose([ 0.325471850974-0.01, -0.182271241593-0.3, 0.0676272396419+0.3 ],
                                       [ -0.000556712307053, -0.706576742941, -0.00102461782513, 0.707635461636 ])
        return self.sample_right_plango(pose_r_target_arbitrary)

    def sample_botharms_plango(self, pose_l, pose_r):
        '''
        Check if http://wiki.ros.org/rtmros_nextage/Tutorials/Programming_Hiro_NEXTAGE_OPEN_MOVEIT works
        '''
        print("=" * 10, " Printing left arm initial pose: ")
        print(pose_l)
        print("=" * 10, " Printing right arm initial pose: ")
        print(pose_r)
        botharm_group = self._armgroup_plango(self._MOVEGROUP_ATTR_BOTHARMS[2], pose_l, 'LARM_JOINT5_Link')
        return self._armgroup_plango(botharm_group, pose_r, 'RARM_JOINT5_Link')

    def sample_botharms_plango_1(self):
        '''Both arm initial pose'''
        pose_r_arbitrary_1 = Pose()
        pose_l_arbitrary_1 = Pose()
        q = quaternion_from_euler(0, -math.pi/2,0)

        pose_l_arbitrary_1.position.x = 0.3
        pose_l_arbitrary_1.position.y =-0.1
        pose_l_arbitrary_1.position.z = 0.3
        pose_l_arbitrary_1.orientation.x = q[0]
        pose_l_arbitrary_1.orientation.y = q[1]
        pose_l_arbitrary_1.orientation.z = q[2]
        pose_l_arbitrary_1.orientation.w = q[3]

        pose_r_arbitrary_1.position.x = 0.3
        pose_r_arbitrary_1.position.y = 0.1
        pose_r_arbitrary_1.position.z = 0.0
        pose_r_arbitrary_1.orientation.x = q[0]
        pose_r_arbitrary_1.orientation.y = q[1]
        pose_r_arbitrary_1.orientation.z = q[2]
        pose_r_arbitrary_1.orientation.w = q[3]

        return self.sample_botharms_plango(pose_l_arbitrary_1, pose_r_arbitrary_1)

    def sample_botharms_plango_2(self):
        '''Both arm arbitrary pose'''
        pose_r_arbitrary_1 = Pose()
        pose_l_arbitrary_1 = Pose()
        q = quaternion_from_euler(0, -math.pi/2,0)

        pose_l_arbitrary_1.position.x = 0.3
        pose_l_arbitrary_1.position.y = 0.2
        pose_l_arbitrary_1.position.z = 0.0

        pose_r_arbitrary_1.position.x = 0.3
        pose_r_arbitrary_1.position.y =-0.2
        pose_r_arbitrary_1.position.z = 0.0

        return self.sample_botharms_plango(pose_l_arbitrary_1, pose_r_arbitrary_1)
