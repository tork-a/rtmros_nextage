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
from hrpsys import rtm

from hironx_ros_bridge import hironx_client as hironx

_ARMGROUP_TESTED = 'larm'
_GOINITIAL_TIME_MIDSPEED = 3  # second
_JOINT_TESTED = (14, 'LARM_JOINT5')  # (%INDEX_IN_LINKGROUP%, %NAME_JOINT%)
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

    #TODO: rostest (ie. boot from .launch file) not passing with this script
    #      for some reasons. Unless this test requires other nodes running,
    #      we might not need to run this from .launch?

    #TODO: Merge with the existing unittest scripts in hironx_ros_bridge/test
    '''

    @classmethod
    def setUpClass(self):
        # Robot info tentatively embedded.
        modelfile = '/opt/jsk/etc/NEXTAGE/model/main.wrl'
        rtm.nshost = 'nxc100'
        robotname = "RobotHardware0"

        self._robot = hironx.HIRONX()
        self._robot.init(robotname=robotname, url=modelfile)
        #TODO: If servo is off, return failure and inform user.

    @classmethod
    def tearDownClass(cls):
        cls._robot.goOffPose()
        True

    def test_setJointAngle_within_pi(self):
        # TODO: setJointAngle does not work as expected for me on
        # hrpsys_simulator. Therefore make this case always fail for now..
        self.assertFalse(True)

    def test_setJointAngle_beyond_pi(self):
        # TODO: setJointAngle does not work as expected for me on
        # hrpsys_simulator. Therefore make this case always fail for now..
        self.assertFalse(True)

    def test_getCurrentPose(self):
        self._robot.goInitial(_GOINITIAL_TIME_MIDSPEED)
        pos = self._robot.getCurrentPose(_JOINT_TESTED[1])
        rospy.loginfo('test_getCurrentPose compare pos={},' +
                      '\n\tPOSE_LARM_JOINT5={}'.format(pos, _POSE_LARM_JOINT5))
        self.assertEqual(pos, _POSE_LARM_JOINT5)

    def test_getCurrentPosition(self):
        self._robot.goInitial(_GOINITIAL_TIME_MIDSPEED)
        pose = self._robot.getCurrentPose(_JOINT_TESTED[1])
        position_from_getcurrentposition = self._robot.getCurrentPosition(
            _JOINT_TESTED[1])
        position_from_pose = [i for j,
                              i in enumerate(pose) if j in _INDICES_POSITION]
        rospy.loginfo('test_getCurrentPosition compare position_from_getcurrentposition={},' +
                      '\n\tposition_from_pose={}'.format(
                          position_from_getcurrentposition, position_from_pose))
        self.assertEqual(position_from_getcurrentposition, position_from_pose)

    def test_getCurrentRotation(self):
        self._robot.goInitial(_GOINITIAL_TIME_MIDSPEED)
        rot_matrix = self._robot.getCurrentRotation(_JOINT_TESTED[1])
        #TODO: Come up with ways to assert the returned value.

    def test_getCurrentRPY(self):
        self._robot.goInitial(_GOINITIAL_TIME_MIDSPEED)
        rpy = self._robot.getCurrentRPY(_JOINT_TESTED[1])
        #TODO: Come up with ways to assert the returned value.

    def test_getJointAngles(self):
        self._robot.goInitial(_GOINITIAL_TIME_MIDSPEED)
        joint_angles = self._robot.getJointAngles()

        # TODO: statically assigning LARM_JOINT5
        self.assertEqual(self._robot.InitialPose[3][5],
                         joint_angles[_JOINT_TESTED[0]])

if __name__ == '__main__':
    import rostest
    rostest.rosrun(_PKG, 'test_hironx_derivedmethods',
                   TestHironxDerivedmethodsFromHrpsys)
