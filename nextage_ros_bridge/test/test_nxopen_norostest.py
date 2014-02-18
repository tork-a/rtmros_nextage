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
from nextage_ros_bridge import nextage_client

from hrpsys import rtm
import argparse

_ARMGROUP_TESTED = 'rarm'
_LINK_TESTED = 'RARM_JOINT5'


class TestNextageNoRostest(object):
    '''
    Test NextageClient without relying on rostest.
    This can be upgraded to rostest version that is more ideal/almost required
    for ROS package.
    '''

    def __init__(self, nc):
        '''
        @type nc: NextageClient
        '''
        self._nxc = nc

#    def test_set_relative_x(self):
    def test_set_relative(self, dx=0, dy=0, dz=0):
        #print('Start moving dx={0}, dy={0}, dz={0}'.format(dx, dy, dz))
        self._nxc.seq_svc.setMaxIKError(0.00001, 0.01)
        posi_prev = self._nxc.getCurrentPosition(_LINK_TESTED)
        for i in range(200):
            self._nxc.setTargetPoseRelative(_ARMGROUP_TESTED, _LINK_TESTED,
                                            dx, dy, dz, tm=0.2)
            #print('   joint=', nxc.getJointAngles()[3:9])
        posi_post = self._nxc.getCurrentPosition(_LINK_TESTED)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='hiro command line interpreters')
    parser.add_argument('--host', help='corba name server hostname')
    parser.add_argument('--port', help='corba name server port number')
    parser.add_argument('--modelfile', help='robot model file nmae')
    parser.add_argument('--robot', help='robot modlule name (RobotHardware0 for real robot, Robot()')
    args, unknown = parser.parse_known_args()

    if args.host:
        rtm.nshost = args.host
    if args.port:
        rtm.nsport = args.port
    if not args.robot:
        args.robot = "RobotHardware0" if args.host else "HiroNX(Robot)0"
    if not args.modelfile:
        args.modelfile = ""

    # support old style format
    if len(unknown) >= 2:
        args.robot = unknown[0]
        args.modelfile = unknown[1]
    nxc = nextage_client.NextageClient()
    nxc.init(robotname=args.robot, url=args.modelfile)
    nxc.goInitial()
    tnn = TestNextageNoRostest(nxc)
#    tnn.test_set_relative_x()
    tnn.test_set_relative(dx=0.0001)
    tnn.test_set_relative(dy=0.0001)
    tnn.test_set_relative(dz=0.0001)
