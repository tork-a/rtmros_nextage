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


class HandlightCommand(AbsractHandCommand):
    '''
    Following Command design pattern, this class represents commands
    for turning hand lights.
    '''
    # TODO: Unittest is needed!!

    HANDLIGHT_ON = True
    HANDLIGHT_OFF = False

    def __init__(self, hands, hand):
        super(HandlightCommand, self).__init__(hands, hand)

    def _assign_dio_names(self):
        '''
        @see abs_hand_command.AbsractHandCommand._assign_dio_names
        '''
        self._DIO_RHAND = self._DIO_17
        self._DIO_LHAND = self._DIO_18

    def execute(self, operation):
        '''
        @see abs_hand_command.AbsractHandCommand.execute

        @param operation: param type:
                          - 'True': Turn the light on.
                          - 'False': Turn the light off.
        '''
        dout = []
        mask = []
        if self.HANDLIGHT_ON == operation:
            if self._hands.HAND_R == self._hand:
                dout = mask = [self._DIO_RHAND]
            elif self._hands.HAND_L == self._hand:
                dout = mask = [self._DIO_LHAND]
            elif not self._hand:  # Both hands
                dout = mask = [self._DIO_RHAND, self._DIO_LHAND]
        else:  # Turn off the light.
            if self._hands.HAND_R == self._hand:
                mask = [self._DIO_RHAND]
            elif self._hands.HAND_L == self._hand:
                mask = [self._DIO_LHAND]
            elif not self._hand:
                mask = [self._DIO_RHAND, self._DIO_LHAND]
        self._hands._dio_writer(dout, mask)
