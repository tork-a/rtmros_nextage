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

import sys

import rospy
from hironx_ros_bridge.hironx_client import HIRONX

from nextage_ros_bridge.iros13_hands import Iros13Hands
from nextage_ros_bridge.hands_05 import Hands05


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

    HAND_VER_0_4_2 = '0.4.2'
    HAND_VER_0_5_1 = '0.5.1'

    use_gazebo = False

    def __init__(self):
        '''
        Do not get confused that there is also a method called
        'init' (without trailing underscores) that is succeeded from the
        super class as the tradition there.
        '''
        try:
            if rospy.has_param('/gazebo'):
                self.use_gazebo = True
                print("\033[32m[INFO] Assuming Gazebo Simulator, so do not connect to CORBA systmes\033[0m")
        # When there's no ros master running, underlining code,
        # socket.create_connection more specifically, returns the simplest
        # 'error' that is hard to be explicitly caught at exception block.
        # See https://github.com/tork-a/rtmros_nextage/issues/262#issue-182487549
        except:
            print("ROS Master not found running. If you intended to use it (either for rosbridge or for Gazebo purposes, make sure you run necessary nodes.")

        if not self.use_gazebo:
            super(NextageClient, self).__init__()
        else:
            self.configurator_name = "gazebo(Nextage)"
        self.set_hand_version(self.HAND_VER_0_5_1)

    def init(self, robotname="HiroNX(Robot)0", url=""):
        '''
        Calls init from its superclass, which tries to connect RTCManager,
        looks for ModelLoader, and starts necessary RTC components. Also runs
        config, logger.
        Also internally calls setSelfGroups().

        @type robotname: str
        @type url: str
        '''
        if not self.use_gazebo:
            HIRONX.init(self, robotname=robotname, url=url)

    def get_hand_version(self):
        '''
        @rtype: str
        '''
        if not self._hand_version:
            return 'Hand module not set yet.'
        else:
            return self._hand_version

    def set_hand_version(self, version=HAND_VER_0_5_1):
        self._hand_version = version
        if self.HAND_VER_0_4_2 == self._hand_version:
            self._hands = Iros13Hands(self)
        elif self.HAND_VER_0_5_1 == self._hand_version:
            self._hands = Hands05(self)

    def _print_msg_deprecated_dio_methods(self, name_method=None):
        '''
        @type name_method: str
        @param name_method: Name of the method that calls this method if available.
        '''
        rospy.logerr("You're likely to have called a deprecated method." +
                     " Use self._hands.%S instead", name_method)

    def handlight_r(self, is_on=True):
        '''
        @deprecated: Won't be functional after package version 0.5.1.
                     Use self._hands.%FUNCTION_NAME% instead.
        '''
        try:
            return self._hands.handlight_r(is_on)
        except AttributeError:
            self._print_msg_deprecated_dio_methods(sys._getframe().f_code.co_name)

    def handlight_l(self, is_on=True):
        '''
        @deprecated: Won't be functional after package version 0.5.1.
                     Use self._hands.%FUNCTION_NAME% instead.
        '''
        try:
            return self._hands.handlight_r(is_on)
        except AttributeError:
            self._print_msg_deprecated_dio_methods(sys._getframe().f_code.co_name)

    def handlight_both(self, is_on=True):
        '''
        @deprecated: Won't be functional after package version 0.5.1.
                     Use self._hands.%FUNCTION_NAME% instead.
        '''
        try:
            return self._hands.handlight_both(is_on)
        except AttributeError:
            self._print_msg_deprecated_dio_methods(sys._getframe().f_code.co_name)

    def handtool_l_eject(self):
        '''
        @deprecated: Won't be functional after package version 0.5.1.
                     Use self._hands.%FUNCTION_NAME% instead.
        '''
        try:
            return self._hands.toolchanger_l_command.execute(
                     self._hands.toolchanger_l_command.HAND_TOOLCHANGE_OFF)
        except AttributeError:
            self._print_msg_deprecated_dio_methods(sys._getframe().f_code.co_name)

    def handtool_r_eject(self):
        '''
        @deprecated: Won't be functional after package version 0.5.1.
                     Use self._hands.%FUNCTION_NAME% instead.
        '''
        try:
            return self._hands.toolchanger_r_command.execute(
                       self._hands.toolchanger_r_command.HAND_TOOLCHANGE_OFF)
        except AttributeError:
            self._print_msg_deprecated_dio_methods(sys._getframe().f_code.co_name)

    def handtool_l_attach(self):
        '''
        @deprecated: Won't be functional after package version 0.5.1.
                     Use self._hands.%FUNCTION_NAME% instead.
        '''
        try:
            return self._hands.toolchanger_l_command.execute(
                self._hands.toolchanger_l_command.HAND_TOOLCHANGE_ON)
        except AttributeError:
            self._print_msg_deprecated_dio_methods(sys._getframe().f_code.co_name)

    def handtool_r_attach(self):
        '''
        @deprecated: Won't be functional after package version 0.5.1.
                     Use self._hands.%FUNCTION_NAME% instead.
        '''
        try:
            return self._hands.toolchanger_r_command.execute(
                self._hands.toolchanger_r_command.HAND_TOOLCHANGE_ON)
        except AttributeError:
            self._print_msg_deprecated_dio_methods(sys._getframe().f_code.co_name)

    def gripper_l_close(self):
        '''
        @deprecated: Won't be functional after package version 0.5.1.
                     Use self._hands.%FUNCTION_NAME% instead.
        '''
        try:
            return self._hands.gripper_l_command.execute(
                self._hands.gripper_l_command.GRIPPER_CLOSE)
        except AttributeError:
            self._print_msg_deprecated_dio_methods(sys._getframe().f_code.co_name)

    def gripper_r_close(self):
        '''
        @deprecated: Won't be functional after package version 0.5.1.
                     Use self._hands.%FUNCTION_NAME% instead.
        '''
        try:
            return self._hands.gripper_r_command.execute(
                self._hands.gripper_r_command.GRIPPER_CLOSE)
        except AttributeError:
            self._print_msg_deprecated_dio_methods(sys._getframe().f_code.co_name)

    def gripper_l_open(self):
        '''
        @deprecated: Won't be functional after package version 0.5.1.
                     Use self._hands.%FUNCTION_NAME% instead.
        '''
        try:
            return self._hands.gripper_l_command.execute(
                self._hands.gripper_r_command.GRIPPER_OPEN)
        except AttributeError:
            self._print_msg_deprecated_dio_methods(sys._getframe().f_code.co_name)

    def gripper_r_open(self):
        '''
        @deprecated: Won't be functional after package version 0.5.1.
                     Use self._hands.%FUNCTION_NAME% instead.
        '''
        try:
            return self._hands.gripper_r_command.execute(
                self._hands.gripper_r_command.GRIPPER_OPEN)
        except AttributeError:
            self._print_msg_deprecated_dio_methods(sys._getframe().f_code.co_name)

    def airhand_l_drawin(self):
        '''
        @deprecated: Won't be functional after package version 0.5.1.
                     Use self._hands.%FUNCTION_NAME% instead.
        '''
        try:
            return self._hands.airhand_l_command.execute(
                self._hands.airhand_l_command.AIRHAND_DRAWIN)
        except AttributeError:
            self._print_msg_deprecated_dio_methods(sys._getframe().f_code.co_name)

    def airhand_r_drawin(self):
        '''
        @deprecated: Won't be functional after package version 0.5.1.
                     Use self._hands.%FUNCTION_NAME% instead.
        '''
        try:
            return self._hands.airhand_r_command.execute(
                self._hands.airhand_r_command.AIRHAND_DRAWIN)
        except AttributeError:
            self._print_msg_deprecated_dio_methods(sys._getframe().f_code.co_name)

    def airhand_l_keep(self):
        '''
        @deprecated: Won't be functional after package version 0.5.1.
                     Use self._hands.%FUNCTION_NAME% instead.
        '''
        try:
            return self._hands.airhand_l_command.execute(
                self._hands.airhand_l_command.AIRHAND_KEEP)
        except AttributeError:
            self._print_msg_deprecated_dio_methods(sys._getframe().f_code.co_name)

    def airhand_r_keep(self):
        '''
        @deprecated: Won't be functional after package version 0.5.1.
                     Use self._hands.%FUNCTION_NAME% instead.
        '''
        try:
            return self._hands.airhand_r_command.execute(
                self._hands.airhand_r_command.AIRHAND_KEEP)
        except AttributeError:
            self._print_msg_deprecated_dio_methods(sys._getframe().f_code.co_name)

    def airhand_l_release(self):
        '''
        @deprecated: Won't be functional after package version 0.5.1.
                     Use self._hands.%FUNCTION_NAME% instead.
        '''
        try:
            return self._hands.airhand_l_command.execute(
                self._hands.airhand_l_command.AIRHAND_RELEASE)
        except AttributeError:
            self._print_msg_deprecated_dio_methods(sys._getframe().f_code.co_name)

    def airhand_r_release(self):
        '''
        @deprecated: Won't be functional after package version 0.5.1.
                     Use self._hands.%FUNCTION_NAME% instead.
        '''
        try:
            return self._hands.airhand_r_command.execute(
                self._hands.airhand_r_command.AIRHAND_RELEASE)
        except AttributeError:
            self._print_msg_deprecated_dio_methods(sys._getframe().f_code.co_name)

    def initialize_hand_dio(self):
        '''
        @deprecated: Won't be functional after package version 0.5.1.
                     Use self._hands.%FUNCTION_NAME% instead.

        Reset all DIO channels to "off" state except for toolchanger lockers
        (if they are turned off the attached tools will fall).
        '''
        self._hands.init_dio()

    def getRTCList(self):
        '''
        Overwriting HrpsysConfigurator.getRTCList
        Returning predefined list of RT components.

        As of March 2016, this method internally calls HIRONX.getRTCList() and
        returns what it returns. Although we could simply remove this method
        from NextageClient, we still keep it; it'd be easier
        to modify an existing method than to add a new overridden method,
        in case we might want to define RTC return list differently from HIRONX.

        @rtype [[str]]
        @rerutrn List of available components. Each element consists of a list
                 of abbreviated and full names of the component.
        '''
        return HIRONX.getRTCList(self)

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

