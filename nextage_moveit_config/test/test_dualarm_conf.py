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

import sys
import unittest

import rospy

from dualarm_conf import DualArmConf, MoveGroupAttr


class TestDualarmConf(unittest.TestCase):
    _PARAM_CONF_DUALARM = 'conf_dualarm'
    _PARAM_SRDF = 'robot_description_semantic'

    @classmethod
    def setUpClass(self):
        True

    @classmethod
    def tearDownClass(self):
        True  # TODO impl something meaningful

    def test_load_config(self):
        '''
        '''
        # Obtain conf yaml and SRDF xml parameters. Test exits if either is unavailable.
        dualarm_conf_file = None
        srdf_xml_text = None
        try:
            dualarm_conf_file = rospy.get_param(self._PARAM_CONF_DUALARM)
        except KeyError:
            rospy.logerr('ROS parameter {} not found. Exiting.'.format(self._PARAM_CONF_DUALARM))
            #sys.exit()
        try:
            srdf_xml_text = rospy.get_param(self._PARAM_SRDF)
        except KeyError:
            rospy.logerr('ROS parameter {} not found. Make sure %YOURPKG_moveit_config%/launch/planning_context.launch is run with `load_robot_description` arg true. Exiting.'.format(self._PARAM_SRDF))
            #sys.exit()

        config = DualArmConf(dualarm_conf_file, srdf_xml_text)
        self.assertIsNotNone(config)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('nextage_ros_bridge', 'test_duarlarm_conf', TestDualarmConf) 
