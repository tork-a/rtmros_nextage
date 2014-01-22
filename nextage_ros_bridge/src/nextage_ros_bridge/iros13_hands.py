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

from nextage_ros_bridge.base_toolchanger_hands import BaseToolchangerHands
from nextage_ros_bridge.command.airhand_command import AbsractHandCommand
from nextage_ros_bridge.command.airhand_command import AirhandCommand
from nextage_ros_bridge.command.gripper_command import GripperCommand


class Iros13Hands(BaseToolchangerHands):
    '''
    This class holds methods to operate the hands of NEXTAGE OPEN in a specific
    configuration where a clipping hand on left and suction hand on right
    connected to toolchanger module. This was presented at IROS 2013.
    '''
    # TODO: Unittest is needed!!

    def __init__(self, parent):
        '''
        @see: AbsractIros13Hands.__init__
        '''

        # Instantiating public command classes.
        #
        # There can be at least 2 ways for the Robot client classes
        # (eg. See nextage_client.NextageClient) to call hand commands.
        #
        #  (1) Call directly the 'execute' methods of each command.
        #  (2) Call via interface methods in the hand class.
        #
        # In Iros13Hands class, (1) is used. (1) is faster to implement. And
        # I think it's ok for robot client classes to know what commands exist,
        # without the hand class has the interfaces for comamands. But you can
        # always apply (2) approach, which is cleaner.
        super(Iros13Hands, self).__init__(parent)
        self.airhand_l_command = AirhandCommand(self, self.HAND_L)
        self.airhand_r_command = AirhandCommand(self, self.HAND_R)
        self.gripper_l_command = GripperCommand(self, self.HAND_L)
        self.gripper_r_command = GripperCommand(self, self.HAND_R)
