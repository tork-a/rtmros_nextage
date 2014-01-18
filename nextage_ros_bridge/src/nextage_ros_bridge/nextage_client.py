#!/usr/bin/env python

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

from hironx_ros_bridge.hironx_client import HIRONX

from nextage_ros_bridge.nextage_hand import NextageHand


class NextageClient(HIRONX, object):
    # The 2nd arg 'object' is passed to work around the issue raised because
    # HrpsysConfigurator is "old-style" python class.
    # See http://stackoverflow.com/a/18392639/577001
    '''
    This class holds methods that are specific to Kawada Industries' dual-arm
    robot called Nextage Open.
    '''
    # TODO: Unittest s'il vous plait!

    ''' Overriding a variable in the superclass to set the arms at higher
    positions.'''
    OffPose = [[0], [0, 0],
                   [25, -140, -150, 45, 0, 0],
                   [-25, -140, -150, -45, 0, 0],
                   [0, 0, 0, 0],
                   [0, 0, 0, 0]]
    InitialPose = [[0], [0, 0],
                   [-0, 0, -130, 0, 0, 0],
                   [0, 0, -130, 0, 0, 0],
                   [0, 0, 0, 0],
                   [0, 0, 0, 0]]

    def __init__(self):
        '''
        Do not get confused that there is also a method called
        'init' (without trailing underscores) that is succeeded from the
        super class as the tradition there.
        '''
        super(NextageClient, self).__init__()
        self._hand = NextageHand(self)

    def init(self, robotname="HiroNX(Robot)0", url=""):
        '''
        Calls init from its superclass, which tries to connect RTCManager,
        looks for ModelLoader, and starts necessary RTC components. Also runs
        config, logger.
        Also internally calls setSelfGroups().

        @type robotname: str
        @type url: str
        '''
        HIRONX.init(self, robotname=robotname, url=url)

    def turn_handlight_r(self, is_on=True):
        self._hand.turn_handlight(self._hand.HAND_R, is_on)

    def turn_handlight_l(self, is_on=True):
        self._hand.turn_handlight(self._hand.HAND_L, is_on)

    def turn_handlight_both(self, is_on=True):
        self._hand.turn_handlight(None, is_on)

    def handtool_eject_l(self):
        self._hand.use_gripper(self._hand.GRIPPER_TOOLCHANGE_OFF,
                               self._hand.HAND_L)

    def handtool_eject_r(self):
        self._hand.use_gripper(self._hand.GRIPPER_TOOLCHANGE_OFF,
                               self._hand.HAND_R)

    def handtool_attach_l(self):
        self._hand.use_gripper(self._hand.GRIPPER_TOOLCHANGE_ON,
                               self._hand.HAND_L)

    def handtool_attach_r(self):
        self._hand.use_gripper(self._hand.GRIPPER_TOOLCHANGE_ON,
                               self._hand.HAND_R)

    def gripper_close_l(self):
        self._hand.use_gripper(self._hand.GRIPPER_CLOSE,
                               self._hand.HAND_L)

    def gripper_close_r(self):
        self._hand.use_gripper(self._hand.GRIPPER_CLOSE,
                               self._hand.HAND_R)

    def gripper_open_l(self):
        self._hand.use_gripper(self._hand.GRIPPER_OPEN,
                               self._hand.HAND_L)

    def gripper_open_r(self):
        self._hand.use_gripper(self._hand.GRIPPER_OPEN,
                               self._hand.HAND_R)

    def airhand_drawin_l(self):
        self._hand.use_gripper(self._hand.AIRHAND_DRAWIN,
                               self._hand.HAND_L)

    def airhand_drawin_r(self):
        self._hand.use_airhand(self._hand.AIRHAND_DRAWIN,
                               self._hand.HAND_R)

    def airhand_keep_l(self):
        self._hand.use_airhand(self._hand.AIRHAND_KEEP,
                               self._hand.HAND_L)

    def airhand_keep_r(self):
        self._hand.use_airhand(self._hand.AIRHAND_KEEP,
                               self._hand.HAND_R)

    def airhand_release_l(self):
        self._hand.use_airhand(self._hand.AIRHAND_RELEASE,
                               self._hand.HAND_L)

    def airhand_release_r(self):
        self._hand.use_airhand(self._hand.AIRHAND_RELEASE,
                               self._hand.HAND_R)

    def initialize_hand_dio(self):
        '''
        Reset all DIO channels to "off" state except for toolchanger lockers
        (if they are turned off the attached tools will fall).
        '''
        self._hand.init_dio()

    #1/15/2014 130s debug
    def getRTCList(self):
        '''
        Overwriting HrpsysConfigurator.getRTCList
        Returning predefined list of RT components.
        @rtype [[str]]
        @rerutrn List of available components. Each element consists of a list
                 of abbreviated and full names of the component.
        '''
        return [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            ['el', "SoftErrorLimiter"],
#            # ['co', "CollisionDetector"],
            ['sc', "ServoController"],
            ['log', "DataLogger"]
            ]

