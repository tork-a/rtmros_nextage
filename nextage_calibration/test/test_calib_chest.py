#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2017, TORK (Tokyo Opensource Robotics Kyokai Association) 
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

import numpy
import unittest

import rospy
import tf


class TestNxoCalib(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_calib')
        rospy.sleep(10)  # intentionally wait for all nodes to start up.

        self.tflistner = tf.TransformListener()

    def tearDown(self):
        True  #TODO impl something meaningful

    def test_tf_head_checkerboard_chest(self):
        '''
        @summary: Check if the translation between 2 frames, head and
                  the camera mounted on top of the head, fall into the intended
                  range. See the image if you want to get an idea of these 2
                  frames: https://cloud.githubusercontent.com/assets/3119480/22644078/8858459c-eca4-11e6-9fb5-4b624d79d499.png
        '''
        FRAME_CAMERA = "/camera_link"
        FRAME_HEAD = "/HEAD_JOINT1_Link"
        TRANS_EXPECTED = [0.117, 0.019, 0.168]
        QUARTERNION_EXPECTED = [-0.001, 0.704, 0.001, 0.710]
        RAD_EXPECTED = [0.008, 1.563, 0.012]
        DEG_EXPECTED = [0.485, 89.550, 0.695]

        self.assertTrue(self.tflistner.frameExists(FRAME_HEAD))
        self.assertTrue(self.tflistner.frameExists(FRAME_CAMERA))

        # To have the following test pass, the camera frame should better fixed
        # onto the robot's body (as of Apr 2017, it's just spawned into Gazebo
        # world without being fixed).
        ##t = self.tflistner.getLatestCommonTime(FRAME_HEAD, FRAME_CAMERA)
        ##pos, quaternion = self.tf.lookupTransform(FRAME_HEAD, FRAME_CAMERA, t)
        ##numpy.testing.assert_almost_equal(pos, TRANS_EXPECTED, 2)
        ##numpy.testing.assert_almost_equal(quaternion, QUARTERNION_EXPECTED, 2)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('nextage_calibration', 'test_calib', TestNxoCalib) 
