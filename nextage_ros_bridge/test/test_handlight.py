#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
# Author: Isaac I.Y. Saito

import unittest

import rostest

from nextage_ros_bridge import nextage_client

_GOINITIAL_TIME_MIDSPEED = 3  # second
_PKG = 'nextage_ros_bridge'


class TestNxoHandlight(unittest.TestCase):
    '''
    Test NextageClient with rostest. This does NOT test hardware (i.e. if DIO
    is connected and functioning); instead, this only verifies if the
    software works as to the given hardware spec.
    
    For tests involving hardware, follow
    https://github.com/start-jsk/rtmros_hironx/issues/272.
    '''

    @classmethod
    def setUpClass(cls):
        cls._robot = nextage_client.NextageClient()
        cls._robot.init()
        cls._robot.goInitial(_GOINITIAL_TIME_MIDSPEED)

        # For older DIO version robot.
        cls._robot_04 = nextage_client.NextageClient()
        cls._robot_04.set_hand_version(version=cls._robot_04.HAND_VER_0_4_2)
        cls._robot_04.init()
        cls._robot_04.goInitial(_GOINITIAL_TIME_MIDSPEED)

    @classmethod
    def tearDownClass(cls):
        cls._robot.goInitial(_GOINITIAL_TIME_MIDSPEED)
        cls._robot_04.goInitial(_GOINITIAL_TIME_MIDSPEED)        

    # On simulator, return of DIO methods should fail no matter what
    # argument is passed. So assertFalse doesn't do much...it works
    # to verify that the tested methods "run without error".
            
    def test_handlight_r_02(self):
        if self._robot.simulation_mode:
            result = self._robot._hands.handlight_r(is_on=False)
            self.assertTrue(result)
        else:
            result = self._robot._hands.handlight_r(is_on=True)
            self.assertTrue(result)

    def test_handlight_r_04(self):
        if self._robot.simulation_mode:
            result = self._robot_04.handlight_r(is_on=False)
            self.assertTrue(result)
        else:
            result = self._robot_04.handlight_r(is_on=True)
            self.assertTrue(result)

    def test_handlight_l_02(self):
        if self._robot.simulation_mode:
            result = self._robot._hands.handlight_l(is_on=False)
            self.assertFalse(result)
        else:
            result = self._robot._hands.handlight_l(is_on=True)
            self.assertTrue(result)

    def test_handlight_l_04(self):
        if self._robot.simulation_mode:
            result = self._robot_04.handlight_l(is_on=False)
            self.assertTrue(result)
        else:
            result = self._robot_04.handlight_l(is_on=True)
            self.assertTrue(result)

    def test_handlight_both_02(self):
        if self._robot.simulation_mode:
            # Check if checking false works.
            result = self._robot._hands.handlight_both(is_on=False)
            self.assertFalse(result)
        else:
            result = self._robot._hands.handlight_both(is_on=True)
            self.assertTrue(result)

    def test_handlight_both_04(self):
        if self._robot.simulation_mode:
            # Check if checking false works.
            result = self._robot_04.handlight_both(is_on=False) and result
            self.assertFalse(result)
        else:
            result = self._robot_04.handlight_both(is_on=True) and result
            self.assertTrue(result)

if __name__ == '__main__':
    rostest.rosrun(_PKG, 'test_nxo_handlight', TestNxoHandlight)
