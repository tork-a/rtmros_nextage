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

import rospy


class AbsractHandCommand(object):
    '''
    Following Command design pattern, this class represents an abstract
    command for hand classes of NEXTAGE OPEN.
    '''
    # TODO: Unittest is needed!!DIO_V

    # DIO pin numbers
    _DIO_RHAND = 17
    _DIO_LHAND = 18

    # DIO pin numbers for toolchangeer.
    _DIO_EJECTOR_R_1 = 22
    _DIO_EJECTOR_R_2 = 23
    _DIO_EJECTOR_L_1 = 27
    _DIO_EJECTOR_L_2 = 28

    _DIO_VALVE5PORT_R = 19
    _DIO_VALVE_R_1 = 20  # Not in use since R hand isn't implemented yet.
    _DIO_VALVE_R_2 = 21
    _DIO_VALVE5PORT_L = 24
    _DIO_VALVE_L_1 = 25
    _DIO_VALVE_L_2 = 26

    def __init__(self, hands, hand):
        '''
        @type hands: nextage_ros_bridge.base_hands.BaseHands
        @param dio_writer: the method that writes out to robot's DIO interface.
        @type hand: str
        '''
        self._hands = hands
        self._hand = hand

    def execute(self, operation):
        '''
        Needs overriddedn, otherwise expcetion occurs.

        @type operation: str
        @param operation: name of the operation.

        @raise exception: HrpsysRosBridgeException
        '''
        msg = 'AbsractHandCommand not extended.'
        rospy.logerr(msg)
        raise NotImplementedError(msg)
