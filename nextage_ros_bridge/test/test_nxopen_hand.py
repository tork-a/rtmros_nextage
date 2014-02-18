#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Tokyo Opensource Robotics Kyokai Association
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

# This should come earlier than later import.
# See http://code.google.com/p/rtm-ros-robotics/source/detail?r=6773
import unittest

#from hrpsys import rtm
from nextage_ros_bridge import nextage_client

_ARMGROUP_TESTED = 'larm'
_LINK_TESTED = 'LARM_JOINT5'
_GOINITIAL_TIME_MIDSPEED = 3  # second
_NUM_CARTESIAN_ITERATION = 300
_PKG = 'nextage_ros_bridge'


class TestNextageopenHand(unittest.TestCase):
    '''
    Test NextageClient with rostest.
    '''

    @classmethod
    def setUpClass(self):
        self._robot = nextage_client.NextageClient()
        self._robot.init()
        self._robot.goInitial(_GOINITIAL_TIME_MIDSPEED)

    def test_airhand_release_l(self):
        self._robot.airhand_release_l()

    def test_airhand_release_r(self):
        self._robot.airhand_release_r()

if __name__ == '__main__':
    import rostest
    rostest.rosrun(_PKG, 'test_nxopen_hand', TestNextageopenHand)
