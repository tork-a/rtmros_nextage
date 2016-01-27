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

import unittest

from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
import rospy


class TestNxoGazebo(unittest.TestCase):
    LINKS = ['ground_plane::link', 'NextageOpen::WAIST', 
             'NextageOpen::CHEST_JOINT0_Link',
             'NextageOpen::HEAD_JOINT0_Link', 'NextageOpen::HEAD_JOINT1_Link',
             'NextageOpen::LARM_JOINT0_Link', 'NextageOpen::LARM_JOINT1_Link', 'NextageOpen::LARM_JOINT2_Link', 'NextageOpen::LARM_JOINT3_Link', 'NextageOpen::LARM_JOINT4_Link', 'NextageOpen::LARM_JOINT5_Link',
             'NextageOpen::RARM_JOINT0_Link', 'NextageOpen::RARM_JOINT1_Link', 'NextageOpen::RARM_JOINT2_Link', 'NextageOpen::RARM_JOINT3_Link', 'NextageOpen::RARM_JOINT4_Link', 'NextageOpen::RARM_JOINT5_Link']
    POSES_INIT = [
                  [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
                  [[0.00999999765496, 0.0100000001748, 0.999999999981], [-6.3680527713e-08, -4.34207636726e-07, -4.26723003096e-08, 1.0]],
                  [[0.00999998170171, 0.0100000013662, 0.999999999851], [1.12277850752e-07, 5.14937728835e-07, -1.00979958524e-07, 1.0]],
                  [[0.0100029694796, 0.00999969305024, 1.56949999981], [3.51357922297e-07, 3.26913242519e-06, 2.056008905e-06, 0.999999999992]],
                  [[0.0100029160077, 0.00999969940026, 1.56950000051], [3.31331093273e-07, 2.8387531827e-06, 1.90629970469e-06, 0.999999999994]],
                  [[0.0100014752154, 0.154999876326, 1.37029605966], [-0.13052464797, 7.19545977957e-05, 0.000532837408226, 0.991444918883]],
                  [[0.0100015151071, 0.154999858779, 1.37029605973], [-0.130525746325, 0.00207084844511, 0.000270668978472, 0.991442720419]],
                  [[0.00889030177994, 0.182057872356, 1.10422937424], [-0.0909336885785, -0.708370543852, 0.0936379038651, 0.693667196748]],
                  [[0.00952371518303, 0.17429613226, 1.07525770057], [-0.0908738541996, -0.708378313981, 0.0935788230317, 0.693675075798]],
                  [[0.244471485549, 0.175825166278, 1.07998479219], [-0.0909215630874, -0.708024441539, 0.0935324598138, 0.694036265991]],
                  [[0.335397381073, 0.164233418171, 1.03631489027], [-0.0908975766118, -0.708021202607, 0.0935569318711, 0.694039413667]],
                  [[0.0100007988562, -0.135000112457, 1.37029594595], [0.130526224152, 7.13412290448e-05, -0.000533010606151, 0.991444711327]],
                  [[0.0100008566461, -0.135000133898, 1.37029594362], [0.130526760024, 0.00207002796155, -0.000269701263587, 0.99144258894]],
                  [[0.00889015589383, -0.162057652536, 1.10422918947], [0.0909351373666, -0.708370877693, -0.0936386321047, 0.6936665676]],
                  [[0.00952360369882, -0.154295921301, 1.07525761426], [0.0908750928422, -0.708378603735, -0.0935797622824, 0.693674490927]],
                  [[0.244471366175, -0.155824931148, 1.07998499932], [0.0909227665517, -0.708024717431, -0.0935334684509, 0.694035690948]],
                  [[0.335397309579, -0.144233039917, 1.03631524451], [0.0908987720726, -0.708021461266, -0.0935579254176, 0.694038859297]]]

    def _cb_gz_linkstates(self, data):
        self._linkstates = data

    def __init__(self, *args, **kwargs):
        super(TestNxoGazebo, self).__init__(*args, **kwargs)
        rospy.init_node('test_nxo_gazebo')
        rospy.loginfo("need to wait for finishing go_initial.py (https://github.com/tork-a/rtmros_nextage/pull/223/files#diff-16b25951a50b1e80569929d32a09102bR14)")
        rospy.sleep(3+4+3)
        rospy.loginfo("start test")
        self._subscriber_gz_linkstates = rospy.Subscriber('/gazebo/link_states', LinkStates, self._cb_gz_linkstates)

    @classmethod
    def setUpClass(self):
        rospy.sleep(5)  # intentionally wait for nextage_gazebo.go_initial to be done.

    @classmethod
    def tearDownClass(self):
        True  # TODO impl something meaningful

    def test_go_initial(self):
        '''Check if arms are moved to init pose'''
        _ORDER_PERMISSIBLE = 2
        # Assert list of the links
        self.assertEqual(sorted(self.LINKS), sorted(self._linkstates.name))

        # Assert if joint values are approximate?
        i = 0
        for pose in self._linkstates.pose:
            rospy.loginfo("check pose " + self.LINKS[i])
            # for coord in pose.position:  # pose.position is an instance of `list`
            # For some reasons, `for coord in pose.position` yields error `'Point' object is not iterable`
            self.assertAlmostEqual(self.POSES_INIT[i][0][0], pose.position.x, places = _ORDER_PERMISSIBLE)
            self.assertAlmostEqual(self.POSES_INIT[i][0][1], pose.position.y, places = _ORDER_PERMISSIBLE)
            self.assertAlmostEqual(self.POSES_INIT[i][0][2], pose.position.z, places = _ORDER_PERMISSIBLE)
            self.assertAlmostEqual(self.POSES_INIT[i][1][0], pose.orientation.x, places = _ORDER_PERMISSIBLE)
            self.assertAlmostEqual(self.POSES_INIT[i][1][1], pose.orientation.y, places = _ORDER_PERMISSIBLE)
            self.assertAlmostEqual(self.POSES_INIT[i][1][2], pose.orientation.z, places = _ORDER_PERMISSIBLE)
            i += 1

if __name__ == '__main__':
    import rostest
    rostest.rosrun('nextage_gazebo', 'test_nxo_gz', TestNxoGazebo)
