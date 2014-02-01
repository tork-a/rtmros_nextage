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

from abs_hand_command import AbsractHandCommand


class ToolchangerCommand(AbsractHandCommand):
    '''
    Following Command design pattern, this class represents commands for
    a toolchanger of NEXTAGE OPEN.

    NOTE: 1/31/2014 TODO: Only right hand is implemented for now.
    '''
    # TODO: Unittest is needed!!

    # For grippers
    GRIPPER_TOOLCHANGE_ON = 'toolchange_on'
    GRIPPER_TOOLCHANGE_OFF = 'toolchange_off'

    def __init__(self, hands, hand):
        super(ToolchangerCommand, self).__init__(hands, hand)

    def execute(self, operation):
        '''
        @see abs_hand_command.AbsractHandCommand.execute
        '''
        dout = []
        # To stop the air when tool is off. we need bits that control gripper.
        mask = [self._DIO_VALVE_L_1, self._DIO_VALVE_L_2]
        if self.GRIPPER_TOOLCHANGE_ON == operation:
            if self._hands.HAND_L == self._hand:
                # 10/29/2013 DIO changed. Now '1' is ON for both 5PORT Valves.
                mask = [self._DIO_VALVE5PORT_L]
            elif self._hands.HAND_R == self._hand:
                mask = [self._DIO_VALVE5PORT_R]
        elif self.GRIPPER_TOOLCHANGE_OFF == operation:
            if self._hands.HAND_L == self._hand:
                # 10/29/2013 DIO changed. Now '0' is OFF for both 5PORT Valves.
                # 1/31/2014 DIO changed. Now '1' is OFF for both 5PORT Valves.
                mask.append(self._DIO_VALVE5PORT_L)
                dout = [self._DIO_VALVE5PORT_L]
            elif self._hands.HAND_R == self._hand:
                mask.append(self._DIO_VALVE5PORT_R)
                dout = [self._DIO_VALVE5PORT_R]
        self._hands._dio_writer(dout, mask)

    def release_ejector(self, hand=None, on=True):
        '''
        @deprecated: TODO: need to figure out how this can be used. Until
              then, set derprecated.
        '''
        dout = []
        mask = []
        if on:
            if self.HAND_R == hand:
                # TODO: Make sure if turning both ejectors at once is the right
                #      usage.
                dout = mask = [self._DIO_EJECTOR_R_1, self._DIO_EJECTOR_R_2]
            elif self.HAND_L == hand:
                dout = mask = [self._DIO_EJECTOR_L_1, self._DIO_EJECTOR_L_2]
            elif not hand:
                dout = mask = [self._DIO_EJECTOR_R_1, self._DIO_EJECTOR_R_2,
                        self._DIO_EJECTOR_L_1, self._DIO_EJECTOR_L_2]
        else:
            if self.HAND_R == hand:
                mask = [self._DIO_EJECTOR_R_1, self._DIO_EJECTOR_R_2]
            elif self.HAND_L == hand:
                mask = [self._DIO_EJECTOR_L_1, self._DIO_EJECTOR_L_2]
            elif not hand:
                mask = [self._DIO_EJECTOR_R_1, self._DIO_EJECTOR_R_2,
                               self._DIO_EJECTOR_L_1, self._DIO_EJECTOR_L_2]
        self._hands._dio_writer(dout, mask)
