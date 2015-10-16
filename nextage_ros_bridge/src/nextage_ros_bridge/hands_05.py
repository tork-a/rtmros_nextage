#!/usr/bin/env python

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

from nextage_ros_bridge.base_hands import BaseHands
from nextage_ros_bridge.command.airhand_command import AbsractHandCommand
from nextage_ros_bridge.command.airhand_command import AirhandCommand
from nextage_ros_bridge.command.gripper_command import GripperCommand
from nextage_ros_bridge.command.handlight_command import HandlightCommand
from nextage_ros_bridge.command.toolchanger_command import ToolchangerCommand


class Hands05(BaseHands):
    '''
    This class holds methods to operate the hands of NEXTAGE OPEN, based on the
    specification of the robot made by the manufacturer (Kawada)
    in August 2014 and thereafter.

    For the robot shipped before then, use Iros13Hands.
    '''

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
        # In this class, (1) is used. (1) is faster to implement. And
        # I think it's ok for robot client classes to know what commands exist,
        # without the hand class has the interfaces for commands. But you can
        # always apply (2) approach, which is cleaner.
        super(Hands05, self).__init__(parent)
        _pins_airhand = [self.DIO_27, self.DIO_28, self.DIO_25, self.DIO_26]
        _pins_gripper = [self.DIO_23, self.DIO_24, self.DIO_21, self.DIO_22]
        _pins_handlight = [self.DIO_18, self.DIO_17]
        _pins_toolchanger = [self.DIO_20, self.DIO_27, self.DIO_28,  # L-hand
                             self.DIO_19, self.DIO_25, self.DIO_26]  # R-hand

        self._airhand_l_command = AirhandCommand(self, self.HAND_L, _pins_airhand)
        self._airhand_r_command = AirhandCommand(self, self.HAND_R, _pins_airhand)
        self._gripper_l_command = GripperCommand(self, self.HAND_L, _pins_gripper)
        self._gripper_r_command = GripperCommand(self, self.HAND_R, _pins_gripper)

        # The following lines are moved from BaseToolchangerHands
        self._handlight_l_command = HandlightCommand(self, self.HAND_L, _pins_handlight)
        self._handlight_r_command = HandlightCommand(self, self.HAND_R, _pins_handlight)
        self._toolchanger_l_command = ToolchangerCommand(self, self.HAND_L, _pins_toolchanger)
        self._toolchanger_r_command = ToolchangerCommand(self, self.HAND_R, _pins_toolchanger)

    def airhand_l_drawin(self):
        return self._airhand_l_command.execute(self._airhand_l_command.AIRHAND_DRAWIN)

    def airhand_r_drawin(self):
        return self._airhand_r_command.execute(self._airhand_r_command.AIRHAND_DRAWIN)

    def airhand_l_keep(self):
        return self._airhand_l_command.execute(self._airhand_l_command.AIRHAND_KEEP)

    def airhand_r_keep(self):
        return self._airhand_r_command.execute(self._airhand_r_command.AIRHAND_KEEP)

    def airhand_l_release(self):
        return self._airhand_l_command.execute(self._airhand_l_command.AIRHAND_RELEASE)

    def airhand_r_release(self):
        return self._airhand_r_command.execute(self._airhand_r_command.AIRHAND_RELEASE)

    def gripper_l_close(self):
        return self._gripper_l_command.execute(self._gripper_l_command.GRIPPER_CLOSE)

    def gripper_r_close(self):
        return self._gripper_r_command.execute(self._gripper_r_command.GRIPPER_CLOSE)

    def gripper_l_open(self):
        return self._gripper_l_command.execute(self._gripper_r_command.GRIPPER_OPEN)

    def gripper_r_open(self):
        return self._gripper_r_command.execute(self._gripper_r_command.GRIPPER_OPEN)

    def handtool_l_eject(self):
        return self._toolchanger_l_command.execute(
            self._toolchanger_l_command.HAND_TOOLCHANGE_OFF)

    def handtool_r_eject(self):
        return self._toolchanger_r_command.execute(
            self._toolchanger_r_command.HAND_TOOLCHANGE_OFF)

    def handtool_l_attach(self):
        return self._toolchanger_l_command.execute(
            self._toolchanger_l_command.HAND_TOOLCHANGE_ON)

    def handtool_r_attach(self):
        return self._toolchanger_r_command.execute(
            self._toolchanger_r_command.HAND_TOOLCHANGE_ON)

    def handlight_r(self, is_on=True):
        return self._handlight_r_command.turn_handlight(self.HAND_R, is_on)

    def handlight_l(self, is_on=True):
        return self._handlight_l_command.turn_handlight(self.HAND_L, is_on)

    def handlight_both(self, is_on=True):
        result = self._handlight_l_command.turn_handlight(self.HAND_L, is_on)
        result = self._handlight_r_command.turn_handlight(self.HAND_R, is_on) and result
        return result
