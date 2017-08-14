#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

# Author: Isaac I.Y. Saito

import unittest

from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander, MoveItCommanderException, RobotCommander
from moveit_msgs.msg import RobotTrajectory
import rospy

import math
from tf.transformations import quaternion_from_euler

class TestDualarmMoveit(unittest.TestCase):
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

    _MOVEGROUP_NAME_LEFTARM_TORSO = 'left_arm_torso'
    _JOINTNAMES_LEFTARM_TORSO = _JOINTNAMES_TORSO +_JOINTNAMES_LEFTARM
    _MOVEGROUP_ATTR_LEFTARM_TORSO = [_MOVEGROUP_NAME_LEFTARM_TORSO, _JOINTNAMES_LEFTARM_TORSO]

    _MOVEGROUP_NAME_RIGHTARM_TORSO = 'right_arm_torso'
    _JOINTNAMES_RIGHTARM_TORSO = _JOINTNAMES_TORSO + _JOINTNAMES_RIGHTARM
    _MOVEGROUP_ATTR_RIGHTARM_TORSO = [_MOVEGROUP_NAME_RIGHTARM_TORSO, _JOINTNAMES_RIGHTARM_TORSO]

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
    ## _JOINTNAMES_BOTHARMS = _JOINTNAMES_TORSO +_JOINTNAMES_LEFTARM + _JOINTNAMES_RIGHTARM
    _JOINTNAMES_BOTHARMS = _JOINTNAMES_LEFTARM + _JOINTNAMES_RIGHTARM
    _MOVEGROUP_ATTR_BOTHARMS = [_MOVEGROUP_NAME_BOTHARMS, _JOINTNAMES_BOTHARMS]

    _MOVEGROUP_NAME_UPPERBODY = 'upperbody'
    _JOINTNAMES_UPPERBODY = _JOINTNAMES_TORSO + _JOINTNAMES_HEAD + _JOINTNAMES_LEFTARM + _JOINTNAMES_RIGHTARM
    _MOVEGROUP_ATTR_UPPERBODY = [_MOVEGROUP_NAME_UPPERBODY, _JOINTNAMES_UPPERBODY]

    # List of all MoveGroup set
    _MOVEGROUP_NAMES = sorted([_MOVEGROUP_NAME_TORSO, _MOVEGROUP_NAME_HEAD,
                               _MOVEGROUP_NAME_LEFTARM, _MOVEGROUP_NAME_RIGHTARM,
                               _MOVEGROUP_NAME_LEFTARM_TORSO, _MOVEGROUP_NAME_RIGHTARM_TORSO,
                               _MOVEGROUP_NAME_LEFTHAND, _MOVEGROUP_NAME_RIGHTHAND,
                               _MOVEGROUP_NAME_BOTHARMS, _MOVEGROUP_NAME_UPPERBODY])
    _MOVEGROUP_ATTRS = [_MOVEGROUP_ATTR_TORSO, _MOVEGROUP_ATTR_HEAD,
                        _MOVEGROUP_ATTR_LEFTARM, _MOVEGROUP_ATTR_RIGHTARM,
                        _MOVEGROUP_ATTR_LEFTARM_TORSO, _MOVEGROUP_ATTR_RIGHTARM_TORSO,
                        _MOVEGROUP_ATTR_LEFTHAND, _MOVEGROUP_ATTR_RIGHTHAND,
                        _MOVEGROUP_ATTR_BOTHARMS, _MOVEGROUP_ATTR_UPPERBODY]

    _TARGETPOSE_LEFT = []  # TODO fill this

#    def __init__(self, mg_attrs_larm, mg_attrs_rarm):
#        '''
#        @param mg_attrs_larm: List of MoveGroup attributes.
#               See the member variable for the semantics of the data type.
#        @type mg_attrs_larm: [str, [str]]
#        '''

    @classmethod
    def setUpClass(self):

        rospy.init_node('test_moveit')
        rospy.sleep(5) # intentinally wait for starting up hrpsys

        self.robot = RobotCommander()

        # TODO: Read groups from SRDF file ideally.

        for mg_attr in self._MOVEGROUP_ATTRS:
            mg = MoveGroupCommander(mg_attr[0])
            # Temporary workaround of planner's issue similar to https://github.com/tork-a/rtmros_nextage/issues/170
            mg.set_planner_id(self._KINEMATICSOLVER_SAFE)
            # Allow replanning to increase the odds of a solution
            mg.allow_replanning(True)
            # increase planning time
            mg.set_planning_time(30.0)
            # Append MoveGroup instance to the MoveGroup attribute list.
            mg_attr.append(mg)

    @classmethod
    def tearDownClass(self):
        True  # TODO impl something meaningful

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
        rospy.loginfo('  plan: {}'.format(plan))
        return plan

    def test_list_movegroups(self):
        '''Check if the defined move groups are loaded.'''
        self.assertEqual(self._MOVEGROUP_NAMES, sorted(self.robot.get_group_names()))

    def test_list_activejoints(self):
        '''Check if the defined joints in a move group are loaded.'''
        for mg_attr in self._MOVEGROUP_ATTRS:
            self.assertEqual(mg_attr[1],  # joint groups for a Move Group.
                             sorted(mg_attr[2].get_active_joints()))

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
        rospy.loginfo('Plan: {}'.format(plan))
        return plan

    def test_plan_success(self):
        '''Evaluate plan (RobotTrajectory)'''
        plan = self._plan_leftarm(self._MOVEGROUP_ATTR_LEFTARM[2])
        # TODO Better way to check the plan is valid.
        # Currently the following checks if the number of waypoints is not zero,
        # which (hopefully) indicates that a path is computed.
        self.assertNotEqual(0, plan.joint_trajectory.points)

    def test_planandexecute(self):
        '''
        Evaluate Plan and Execute works.
        Currently no value checking is done (, which is better to be implemented)
        '''
        mvgroup = self._MOVEGROUP_ATTR_LEFTARM[2]
        self._plan_leftarm(mvgroup)
        # TODO Better way to check the plan is valid.
        # The following checks if plan execution was successful or not.
        self.assertTrue(mvgroup.go() or mvgroup.go() or mvgroup.go())

    def test_left_and_right_plan(self):
        '''
        Check if http://wiki.ros.org/rtmros_nextage/Tutorials/Programming_Hiro_NEXTAGE_OPEN_MOVEIT works
        '''
        larm = self._MOVEGROUP_ATTR_LEFTARM[2]
        rarm = self._MOVEGROUP_ATTR_RIGHTARM[2]

        #Right Arm Initial Pose
        rarm_initial_pose = rarm.get_current_pose().pose
        print "=" * 10, " Printing Right Hand initial pose: "
        print rarm_initial_pose

        #Light Arm Initial Pose
        larm_initial_pose = larm.get_current_pose().pose
        print "=" * 10, " Printing Left Hand initial pose: "
        print larm_initial_pose

        target_pose_r = Pose()
        target_pose_r.position.x = 0.325471850974-0.01
        target_pose_r.position.y = -0.182271241593-0.3
        target_pose_r.position.z = 0.0676272396419+0.3
        target_pose_r.orientation.x = -0.000556712307053
        target_pose_r.orientation.y = -0.706576742941
        target_pose_r.orientation.z = -0.00102461782513
        target_pose_r.orientation.w = 0.707635461636
        rarm.set_pose_target(target_pose_r)

        print "=" * 10," plan1 ..."
        self.assertTrue(rarm.go() or rarm.go() or rarm.go())
        rospy.sleep(1)

        target_pose_l = [
                target_pose_r.position.x,
                -target_pose_r.position.y,
                target_pose_r.position.z,
                target_pose_r.orientation.x,
                target_pose_r.orientation.y,
                target_pose_r.orientation.z,
                target_pose_r.orientation.w
        ]
        larm.set_pose_target(target_pose_l)

        print "=" * 10," plan2 ..."
        self.assertTrue(larm.go() or larm.go() or larm.go())
        rospy.sleep(1)

        #Clear pose
        rarm.clear_pose_targets()

        #Right Hand
        target_pose_r.position.x = 0.221486843301
        target_pose_r.position.y = -0.0746407547512
        target_pose_r.position.z = 0.642545484602
        target_pose_r.orientation.x = 0.0669013615474
        target_pose_r.orientation.y = -0.993519060661
        target_pose_r.orientation.z = 0.00834224628291
        target_pose_r.orientation.w = 0.0915122442864
        rarm.set_pose_target(target_pose_r)

        print "=" * 10, " plan3..."
        self.assertTrue(rarm.go() or rarm.go() or rarm.go())
        rospy.sleep(1)

        print "=" * 10,"Initial pose ..."
        rarm.set_pose_target(rarm_initial_pose)
        larm.set_pose_target(larm_initial_pose)
        self.assertTrue(rarm.go() or rarm.go() or rarm.go())
        self.assertTrue(larm.go() or larm.go() or larm.go())

    def test_botharms_plan(self):
        botharms = self._MOVEGROUP_ATTR_BOTHARMS[2]

        target_pose_r = Pose()
        target_pose_l = Pose()
        q = quaternion_from_euler(0, -math.pi/2,0)
        target_pose_r.position.x = 0.3
        target_pose_r.position.y = 0.1
        target_pose_r.position.z = 0.0
        target_pose_r.orientation.x = q[0]
        target_pose_r.orientation.y = q[1]
        target_pose_r.orientation.z = q[2]
        target_pose_r.orientation.w = q[3]
        target_pose_l.position.x = 0.3
        target_pose_l.position.y =-0.1
        target_pose_l.position.z = 0.3
        target_pose_l.orientation.x = q[0]
        target_pose_l.orientation.y = q[1]
        target_pose_l.orientation.z = q[2]
        target_pose_l.orientation.w = q[3]
        botharms.set_pose_target(target_pose_r, 'RARM_JOINT5_Link')
        botharms.set_pose_target(target_pose_l, 'LARM_JOINT5_Link')
        self.assertTrue(botharms.go() or botharms.go() or botharms.go())
        rospy.sleep(1)

        target_pose_r.position.x = 0.3
        target_pose_r.position.y =-0.2
        target_pose_r.position.z = 0.0
        target_pose_l.position.x = 0.3
        target_pose_l.position.y = 0.2
        target_pose_l.position.z = 0.0
        botharms.set_pose_target(target_pose_r, 'RARM_JOINT5_Link')
        botharms.set_pose_target(target_pose_l, 'LARM_JOINT5_Link')
        self.assertTrue(botharms.go() or botharms.go() or botharms.go())

if __name__ == '__main__':
    import rostest
    rostest.rosrun('nextage_ros_bridge', 'test_moveit', TestDualarmMoveit) 
