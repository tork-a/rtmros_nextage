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

    def __init__(self, hands, hand, dio_pins):
        '''
        @type hands: nextage_ros_bridge.base_hands.BaseHands
        @type hand: str
        @param hand: Side of hand. Variables that are defined in
                     nextage_ros_bridge.base_hands.BaseHands can be used
                     { HAND_L, HAND_R }.
        @type dio_pins: [int]
        @param dio_pins: List of DIO pins that are used in each HandCommand
                         class. The order is important; it needs be defined
                         in subclasses.
        '''
        self._hands = hands
        self._hand = hand
        self._assign_dio_names(dio_pins)

    def execute(self, operation):
        '''
        Needs overriddedn, otherwise expcetion occurs.

        @type operation: str
        @param operation: name of the operation.
        @rtype: bool
        @return: True if dout was writtable to the register. False otherwise.

        @raise exception: RuntimeError
        '''
        msg = 'AbsractHandCommand.execute() not extended.'
        rospy.logerr(msg)
        raise NotImplementedError(msg)

    def _assign_dio_names(self, dio_pins):
        '''
        It's recommended in the derived classes to re-assign DIO names to
        better represent the specific purposes of each DIO pin in there.
        Since doing so isn' mandatory, this method doesn't emit error even when
        it's not implemented.
        @type dio_pins: [int]
        @param dio_pins: List of DIO pins that are used in each HandCommand
                         class. The order is important; it needs be defined
                         in subclasses.
        '''
        pass
