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
# Author: Isaac Isao Saito

# This should come earlier than later import.
# See http://code.google.com/p/rtm-ros-robotics/source/detail?r=6773
import numpy
from threading import Thread
import unittest

import rospy
import tf

from hrpsys import rtm
from nextage_ros_bridge import nextage_client

_ARMGROUP_TESTED = 'larm'
_JOINT_TESTED = 'LARM_JOINT5'
_LINK_TESTED = 'LARM_JOINT5_Link'
_LINK_FOR_TEST = _LINK_TESTED + '_TEMP'
_GOINITIAL_TIME_MIDSPEED = 3  # second
_NUM_CARTESIAN_ITERATION = 300
_PKG = 'nextage_ros_bridge'
_FRAME_ROOT = 'WAIST'


class AdditionalTfPubThread(Thread):
    '''Publish an additional tf frame, in order to test the consistencyof
    model conversion.    
    '''
    def __init__(self, parent, param_name_raw):
        super(AdditionalTfPubThread, self).__init__()
        rospy.init_node('tf_publisher')
        self._tf_broadcaster = tf.TransformBroadcaster()

        self._hiro = nextage_client.NextageClient()
        self._hiro.init('HiroNX(Robot)0', '')

    def _publish_tf(self):
        pos = self.hiro.getCurrentPosition(_JOINT_TESTED)
        pos[2] = pos[2] - self._hiro.getCurrentPosition(_FRAME_ROOT)[2]  # Subtract waist height
        rpy = self.hiro.getReferenceRPY(_JOINT_TESTED)
        self._tf_broadcaster.sendTransform((pos[0], pos[1], pos[2]),
              tf.transformations.quaternion_from_euler(
                    rpy[0], rpy[1], rpy[2]), rospy.Time.now(),
                    "LARM_JOINT5_Link_Controller", _FRAME_ROOT)

    def run(self):
        _rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            self._publish_tf()
            _rate.sleep()
#         except rospy.exceptions.ROSException as e:
#             raise type(e)(e.message +
#                           "TreenodeQstdItem. Couldn't connect to {}".format(self._param_name_raw))

class TestNxoRos(unittest.TestCase):
    '''
    Test NextageClient with rostest.
    '''

    @classmethod
    def setUpClass(self):

        modelfile = '/opt/jsk/etc/HIRONX/model/main.wrl'
        rtm.nshost = 'localhost'
        robotname = "RobotHardware0"

        self._robot = nextage_client.NextageClient()
        self._robot.init(robotname=robotname, url=modelfile)

        self._robot.goInitial(_GOINITIAL_TIME_MIDSPEED)

    def test_tf_coordinate(self):
        '''
        Originally added to test:
        https://github.com/start-jsk/rtmros_hironx/issues/287
        '''

        # Run a new thread to start additional tf publisher.
        addition_tf_thread = AdditionalTfPubThread()
        addition_tf_thread.start()
        
        # Move left arm to certain pose where RPY is rotated enough to show
        # the 2 tf frames not align when there's an issue. 
        self._robot.setTargetPoseRelative(
                _ARMGROUP_TESTED, _JOINT_TESTED,
                dr=0.3, dp=0.25, dw=0.2, tm=2)
        # Get tf for each frame.
        (trans_t1, rot_t1) = tfl.lookupTransform(_LINK_TESTED, _FRAME_ROOT, rospy.Time(0))
        (trans_t2, rot_t2) = tfl.lookupTransform(_LINK_FOR_TEST, _FRAME_ROOT, rospy.Time(0))
        # Check diff
        is_close = True
        is_close = numpy.isclose(trans_t1[0], trans_t2[0], rtol=1e-04, equal_nan=False) and is_close
        is_close = numpy.isclose(trans_t1[1], trans_t2[1], rtol=1e-04, equal_nan=False) and is_close
        is_close = numpy.isclose(trans_t1[2], trans_t2[2], rtol=1e-04, equal_nan=False) and is_close
        is_close = numpy.isclose(rot_t1[0], rot_t2[0], rtol=1e-04, equal_nan=False) and is_close
        is_close = numpy.isclose(rot_t1[1], rot_t2[1], rtol=1e-04, equal_nan=False) and is_close
        is_close = numpy.isclose(rot_t1[2], rot_t2[2], rtol=1e-04, equal_nan=False) and is_close
        assertTrue(is_close)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(_PKG, 'nxopen_withros', TestNxoRos)
