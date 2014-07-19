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

from nextage_ros_bridge.iros13_hands import Iros13Hands


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

    # Default digital input groups defined by manufacturer, Kawada, as of
    # July 2014. This may change per the robot in the future and in then
    # need modified. See also readDinGroup method.
    _DI_PORTS_L = [25, 21, 22, 23, 24]
    _DI_PORTS_R = [20, 16, 17, 18, 19]

    def __init__(self):
        '''
        Do not get confused that there is also a method called
        'init' (without trailing underscores) that is succeeded from the
        super class as the tradition there.
        '''
        super(NextageClient, self).__init__()
        self._hand = Iros13Hands(self)

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

    def handlight_r(self, is_on=True):
        return self._hand.turn_handlight(self._hand.HAND_R, is_on)

    def handlight_l(self, is_on=True):
        return self._hand.turn_handlight(self._hand.HAND_L, is_on)

    def handlight_both(self, is_on=True):
        return self._hand.turn_handlight(None, is_on)

    def handtool_l_eject(self):
        return self._hand.toolchanger_l_command.execute(
            self._hand.toolchanger_l_command.HAND_TOOLCHANGE_OFF)

    def handtool_r_eject(self):
        return self._hand.toolchanger_r_command.execute(
            self._hand.toolchanger_r_command.HAND_TOOLCHANGE_OFF)

    def handtool_l_attach(self):
        return self._hand.toolchanger_l_command.execute(
            self._hand.toolchanger_l_command.HAND_TOOLCHANGE_ON)

    def handtool_r_attach(self):
        return self._hand.toolchanger_r_command.execute(
            self._hand.toolchanger_r_command.HAND_TOOLCHANGE_ON)

    def gripper_l_close(self):
        return self._hand.gripper_l_command.execute(
            self._hand.gripper_l_command.GRIPPER_CLOSE)

    def gripper_r_close(self):
        return self._hand.gripper_r_command.execute(
            self._hand.gripper_r_command.GRIPPER_CLOSE)

    def gripper_l_open(self):
        return self._hand.gripper_l_command.execute(
            self._hand.gripper_r_command.GRIPPER_OPEN)

    def gripper_r_open(self):
        return self._hand.gripper_r_command.execute(
            self._hand.gripper_r_command.GRIPPER_OPEN)

    def airhand_l_drawin(self):
        return self._hand.airhand_l_command.execute(
            self._hand.airhand_l_command.AIRHAND_DRAWIN)

    def airhand_r_drawin(self):
        return self._hand.airhand_r_command.execute(
            self._hand.airhand_r_command.AIRHAND_DRAWIN)

    def airhand_l_keep(self):
        return self._hand.airhand_l_command.execute(
            self._hand.airhand_l_command.AIRHAND_KEEP)

    def airhand_r_keep(self):
        return self._hand.airhand_r_command.execute(
            self._hand.airhand_r_command.AIRHAND_KEEP)

    def airhand_l_release(self):
        return self._hand.airhand_l_command.execute(
            self._hand.airhand_l_command.AIRHAND_RELEASE)

    def airhand_r_release(self):
        return self._hand.airhand_r_command.execute(
            self._hand.airhand_r_command.AIRHAND_RELEASE)

    def initialize_hand_dio(self):
        '''
        Reset all DIO channels to "off" state except for toolchanger lockers
        (if they are turned off the attached tools will fall).
        '''
        self._hand.init_dio()

    # 1/15/2014 130s debug
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
            # ['co', "CollisionDetector"],
            # ['sc', "ServoController"],
            ['log', "DataLogger"]
            ]

    def goInitial(self, tm=7, wait=True, init_pose_type=0):
        '''
        @see: HIRONX.goInitial
        '''
        if not init_pose_type:
            # Set the pose where eefs level with the tabletop by default.
            init_pose_type = HIRONX.INITPOS_TYPE_EVEN
        return HIRONX.goInitial(self, tm, wait, init_pose_type)

    def readDinGroup(self, ports, dumpFlag=True):
        '''
        Print the currently set values of digital input registry. Print output order is tailored 
        for the hands' functional group; DIO spec that is disloseable as of 7/17/2014 is:

             Left hand: 
                  DI26: Tool changer attached or not.
                  DI22, 23: Fingers.
                  DI24, 25: Compliance.

             Right hand: 
                  DI21: Tool changer attached or not.
                  DI17, 18: Fingers.
                  DI19, 20: Compliance.

        Example output, for the right hand: 

            No hand attached:

                In [1]: robot.printDin([20, 16, 17, 18, 19])
                DI21 is 0
                DI17 is 0
                DI18 is 0
                DI19 is 0
                DI20 is 0
                Out[1]: [(20, 0), (16, 0), (17, 0), (18, 0), (19, 0)]
    
            Hand attached, fingers closed:

                In [1]: robot.printDin([20, 16, 17, 18, 19])
                DI21 is 1
                DI17 is 1
                DI18 is 0
                DI19 is 0
                DI20 is 0
                Out[1]: [(20, 0), (16, 0), (17, 0), (18, 0), (19, 0)]
    
        @author: Koichi Nagashima
        @since: 0.2.16
        @type ports: int or [int].
        @param dumpFlag: Print each pin if True.
        @param ports: A port number or a list of port numbers in D-in registry.
        @rtype: [(int, int)]
        @return: List of tuples of port and din value. If the arg ports was an int value, 
                 this could be a list with single tuple in it.
        '''
        if isinstance(ports, int):
            ports = [ports];
            pass;
        #din = self.rh_svc.readDigitalInput()[1];
        ## rh_svc.readDigitalInput() returns tuple, of which 1st element is not needed here.
        din = self.readDigitalInput();
        resAry=[];
        for port in ports:
            res = din[port]
            if (dumpFlag): print("DI%02d is %d"%(port+1,res));
            resAry.append((port, res));
            pass;
        return resAry;

    def readDinGroupL(self, dumpFlag=True):
        return self.readDinGroup(self._DI_PORTS_L, dumpFlag)

    def readDinGroupR(self, dumpFlag=True):
        return self.readDinGroup(self._DI_PORTS_R, dumpFlag)

