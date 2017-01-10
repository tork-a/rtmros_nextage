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

import rospy

from nextage_moveit_config.sample_moveit import SampleMoveitCommander


class TestDualarmMoveit(unittest.TestCase):

#    def __init__(self, mg_attrs_larm, mg_attrs_rarm):
#        '''
#        @param mg_attrs_larm: List of MoveGroup attributes.
#               See the member variable for the semantics of the data type.
#        @type mg_attrs_larm: [str, [str]]
#        '''

    @classmethod
    def setUpClass(self):

        rospy.sleep(5)  # intentinally wait for starting up hrpsys

        self.robot = SampleMoveitCommander()

    @classmethod
    def tearDownClass(self):
        True  # TODO impl something meaningful

    def test_list_movegroups(self):
        '''Check if the defined move groups are loaded.'''
        self.assertEqual(self.robot._MOVEGROUP_NAMES, sorted(self.robot.get_group_names()))

    def test_list_activejoints(self):
        '''Check if the defined joints in a move group are loaded.'''
        for mg_attr in self.robot._MOVEGROUP_ATTRS:
            self.assertEqual(mg_attr[1],  # joint groups for a Move Group.
                             sorted(mg_attr[2].get_active_joints()))

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
        mvgroup = self.robot._MOVEGROUP_ATTR_LEFTARM[2]
        self.robot._plan_leftarm(mvgroup)
        # TODO Better way to check the plan is valid.
        # The following checks if plan execution was successful or not.
        self.assertTrue(mvgroup.go())

    def test_left_and_right_plan(self):
        self.assertTrue(self.robot.sample_left_plango1())
        rospy.sleep(1)

        self.assertTrue(self.robot.sample_right_plango1())
        rospy.sleep(1)

        #Clear pose
        #larm.clear_pose_targets()
        #rarm.clear_pose_targets()

        self.assertTrue(self.robot.sample_left_plango2())
        rospy.sleep(1)

        self.assertTrue(self.robot.sample_right_plango2())
        rospy.sleep(1)

        print "=" * 10, "Initial pose ..."

        self.assertTrue(self.robot.sample_left_plango1())
        self.assertTrue(self.robot.sample_right_plango1())

    def test_botharms_plan(self):
        self.assertTrue(self.robot.sample_botharms_plango_1())
        rospy.sleep(1)
        self.assertTrue(self.robot.sample_botharms_plango_2())

if __name__ == '__main__':
    import rostest
    rostest.rosrun('nextage_ros_bridge', 'test_moveit', TestDualarmMoveit) 
