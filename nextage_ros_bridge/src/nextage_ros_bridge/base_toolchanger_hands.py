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

from nextage_ros_bridge.base_hands import BaseHands
from nextage_ros_bridge.command.toolchanger_command import ToolchangerCommand
from nextage_ros_bridge.command.handlight_command import HandlightCommand


class BaseToolchangerHands(BaseHands):
    '''
    This class holds methods that are specific to the hands of NEXTAGE OPEN,
    accompanied with toolchanger.

    @deprecated: Since version 0.5.1, the functionality in this class is moved
                 to other BaseHands subclasses (e.g. Iros13Hands).
    '''
    # TODO: Unittest is needed!!

    def __init__(self, parent):
        '''
        Since this class operates requires an access to
        hrpsys.hrpsys_config.HrpsysConfigurator, valid 'parent' is a must.
        Otherwise __init__ returns without doing anything.

        @type parent: hrpsys.hrpsys_config.HrpsysConfigurator
        @param parent: derived class of HrpsysConfigurator.
        '''
        super(BaseToolchangerHands, self).__init__(parent)
        if not parent:
            return  # TODO: Replace with throwing exception
        self._parent = parent

        self.handlight_l_command = HandlightCommand(self, self.HAND_L)
        self.handlight_r_command = HandlightCommand(self, self.HAND_R)
        self.toolchanger_l_command = ToolchangerCommand(self, self.HAND_L)
        self.toolchanger_r_command = ToolchangerCommand(self, self.HAND_R)

    def turn_handlight(self, hand=None, on=True):
        '''
        @param hand: Both hands if None.
        @type on: bool
        @param on: Despite its type, it's handled as str in this method.
        @rtype: bool
        @return: True if the lights turned. False otherwise.
        '''
        _result = True
        if self.HAND_L == hand:
            _result = self.handlight_l_command.execute(on)
        elif self.HAND_R == hand:
            _result = self.handlight_r_command.execute(on)
        elif not hand:  # both hands
            _result = self.handlight_l_command.execute(on) and _result
            _result = self.handlight_r_command.execute(on) and _result
        return _result
