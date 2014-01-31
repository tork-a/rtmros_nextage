#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Tokyo Opensource Robotics Kyokai Association
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
#  * Neither the name of Tokyo Opensource Robotics Kyokai Association. nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
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
#
# Author: Isaac Isao Saito

import unittest

import rospy

from hironx_ros_bridge import hironx_client as hironx

_ARMGROUP_TESTED = 'larm'
_GOINITIAL_TIME_MIDSPEED = 3  # second
_JOINT_TESTED = 'LARM_JOINT5'
_PKG = 'nextage_ros_bridge'
_POSE_LARM_JOINT5 = [-0.0017695671419774017, 0.00019009187609567157,
                     -0.99999841624735, 0.3255751238415409,
                     0.00012113080903022877, 0.9999999746368694,
                     0.00018987782289436872, 0.18236314012331808,
                     0.9999984269784911, -0.00012079461563276815,
                     -0.0017695901230784174, 1.037624522677471,
                     0.0, 0.0, 0.0, 1.0]  # At goInitial.
_INDICES_POSITION = 3, 7, 11  # In pose returned by getCurrentPose.


class TestHironxDerivedmethodsFromHrpsys(unittest.TestCase):
    '''
    Test the derived methods of hrpsys_config.HrpsysConfigurator that are
    overridden in hironx_client.HIRONX.

    So far this test can be run by:

        $ rosrun nextage_ros_bridge test_hironx_derivedmethods_rostest.py

    #TODO: rostest not passing for some reasons.
    '''

    @classmethod
    def setUpClass(self):
        self._robot = hironx.HIRONX()
        self._robot.init()

    def test_getCurrentPose(self):
        self._robot.goInitial(_GOINITIAL_TIME_MIDSPEED)
        pos = self._robot.getCurrentPose(_JOINT_TESTED)
        rospy.loginfo('test_getCurrentPose compare pos={},' +
                      '\n\t_POSE_LARM_JOINT5={}'.format(pos, _POSE_LARM_JOINT5))
        self.assertEqual(pos, _POSE_LARM_JOINT5)

    def test_getCurrentPosition(self):
        self._robot.goInitial(_GOINITIAL_TIME_MIDSPEED)
        pose = self._robot.getCurrentPose(_JOINT_TESTED)
        position_from_getcurrentposition = \
                                self._robot.getCurrentPosition(_JOINT_TESTED)
        position_from_pose = [i for j,
                              i in enumerate(pose) if j in _INDICES_POSITION]
        rospy.loginfo('test_getCurrentPosition compare position_from_getcurrentposition={},' +
                      '\n\tposition_from_pose={}'.format(
                         position_from_getcurrentposition, position_from_pose))
        self.assertEqual(position_from_getcurrentposition, position_from_pose)

    def test_getCurrentRPY(self):
        self._robot.goInitial(_GOINITIAL_TIME_MIDSPEED)
        rpy = self._robot.getCurrentRPY(_JOINT_TESTED)
        #TODO: Come up with ways to assert rpy.

if __name__ == '__main__':
    import rostest
    rostest.rosrun(_PKG, 'test_hironx_derivedmethods',
                   TestHironxDerivedmethodsFromHrpsys)
