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

import time
import threading

import rospy


class AirhandReleaseThread(threading.Thread):
    '''
    With airhand, to release by blowing air needs to be stopped after certain
    amount of time. Usually a few seconds, not too short for the air compressor
    to have ample time for a reaction, is a good idea. This thread is used
    in order to do so without letting the computer program halt.
    '''
    def __init__(self, nhand, hand, sleeptime):
        '''
        @type nhand: NextageHand
        @type hand: str
        '''
        threading.Thread.__init__(self)
        self._nhand = nhand
        self._hand = hand
        self._sleeptime = sleeptime

    def run(self):
        time.sleep(self._sleeptime)
        self._nhand.use_airhand(self._nhand.AIRHAND_KEEP, self._hand)


class NextageHand(object):
    '''
    This class holds methods that are specific to the hands of
    Kawada Industries' dual-arm robot called Nextage Open.
    '''
    # TODO: Unittest is needed!!

    HAND_L = '1'  # '0' is expected to be "Both hands".
    HAND_R = '2'

    # For air hands
    AIRHAND_DRAWIN = 'drawin'
    AIRHAND_KEEP = 'keep'
    AIRHAND_RELEASE = 'release'
    ## Might not be necessary. Maybe use only where you have to specify
    ## dangerous situation.
    AIRHAND_DANGER = 'danger'

    # For grippers
    GRIPPER_TOOLCHANGE_ON = 'toolchange_on'
    GRIPPER_TOOLCHANGE_OFF = 'toolchange_off'
    GRIPPER_CLOSE = 'close'
    GRIPPER_OPEN = 'open'
    GRIPPER_DANGER = 'danger'

    # DIO pin numbers
    _DIO_RHAND = 17
    _DIO_LHAND = 18
    _DIO_VALVE5PORT_R = 19
    _DIO_VALVE_R_1 = 20  # Not in use since R hand isn't implemented yet.
    _DIO_VALVE_R_2 = 21
    _DIO_EJECTOR_R_1 = 22
    _DIO_EJECTOR_R_2 = 23
    _DIO_VALVE5PORT_L = 24
    _DIO_VALVE_L_1 = 25
    _DIO_VALVE_L_2 = 26
    _DIO_EJECTOR_L_1 = 27
    _DIO_EJECTOR_L_2 = 28

    _SLEEP_POST_RELEASE = 3.0

    def __init__(self, parent):
        '''
        Since this class operates requires an access to
        hrpsys.hrpsys_config.HrpsysConfigurator, valid 'parent' is a must.
        Otherwise __init__ returns without doing anything.

        @type parent: hrpsys.hrpsys_config.HrpsysConfigurator
        '''
        if not parent:
            return  # TODO: Replace with throwing exception
        self._parent = parent

    def _dio_writer(self, digital_out, dio_assignments, padding=1):
        '''
        This private method calls hrpsys_config.writeDigitalOutputWithMask,
        which this class expects to be available via self._parent, that accepts
        arrays of bits.

        According to the current (Oct 2013) hardware spec, numbering rule
        differs regarding 0 (numeric figure) in dout and mask as follows:

           * 0 is "ON" in the digital output.
           * 0 is "masked" and not used in mask.

        @type digital_out: int[]
        @param digital_out: Array of indices of digital output that NEED to be
                            flagged as 1.
                            Example: If you're targetting on 25 and 26th places
                                     in the DIO array but only 25th is 1, then
                                     the array becomes [24].
        @type dio_assignments: int[]
        @param dio_assignments: range(32). This number corresponds to the
                               assigned digital pin of the robot.

                               Example: If the target pin are 25 and 26,
                                        dio_assignments = [24, 25]
        @param padding: Either 0 or 1. Signal arrays will be filled with this
                        value.
        '''

        # 32 bit arrays used in write methods in hrpsys/hrpsys_config.py
        p = padding
        dout = []
        for i in range(32):
            dout.append(p)
        mask = []
        for i in range(32):
            mask.append(0)

        signal_alternate = 0
        if padding == 0:
            signal_alternate = 1
        for i in digital_out:
            dout[i - 1] = signal_alternate

        for i in dio_assignments:
            # For masking, alternate symbol is always 1.
            mask[i - 1] = 1

        # For convenience only; to show array number.
        print_index = []
        for i in range(10):
            # For masking, alternate symbol is always 1.
            n = i + 1
            if 10 == n:
                n = 0
            print_index.append(n)
        print_index.extend(print_index)
        print_index.extend(print_index)
        del print_index[-8:]

        # # For some reason rospy.loginfo not print anything.
        # rospy.loginfo('dout={}, mask={}'.format(dout, mask))
        # # With this print formatting, you can copy the output and paste
        # # directly into writeDigitalOutputWithMask method if you wish.
        rospy.loginfo('dout, mask:\n{},\n{}\n{}'.format(dout, mask,
                                                        print_index))
        try:
            self._parent.writeDigitalOutputWithMask(dout, mask)
        except AttributeError as e:
            rospy.logerr('AttributeError from robot.\nTODO: Needs handled.')
            rospy.logerr('\t{}'.format("Device was not found. Maybe you're on simulator?"))            

    def turn_handlight(self, hand=None, on=True):
        '''
        @type hand: str
        @param hand: Either "l", "r", or None. If None, turn on the lights on
                     both hands.
        @type on: bool
        @param on: True = on, False = off
        '''
        dout = []
        mask = []
        if on:
            if self.HAND_R == hand:
                dout = mask = [self._DIO_RHAND]
            elif self.HAND_L == hand:
                dout = mask = [self._DIO_LHAND]
            elif not hand:
                dout = mask = [self._DIO_RHAND, self._DIO_LHAND]
        else:  # Turn off the light.
            if self.HAND_R == hand:
                mask = [self._DIO_RHAND]
            elif self.HAND_L == hand:
                mask = [self._DIO_LHAND]
            elif not hand:
                mask = [self._DIO_RHAND, self._DIO_LHAND]
        self._dio_writer(dout, mask)

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
        self._dio_writer(dout, mask)

    def use_gripper(self, operation, hand=None):
        '''
        NOTE: Gripping features are currently implemented for LEFT hand only.

        @type hand: str
        @param hand: Use class member variable HAND_L or R
        '''
        dout = []
        mask = [self._DIO_VALVE_L_1, self._DIO_VALVE_L_2]

        # TODO: Implement for left hand too.
        if self.GRIPPER_TOOLCHANGE_ON == operation:
            if self.HAND_L == hand:
                # 10/29/2013 DIO changed. Now '1' is ON for both 5PORT Valves.
                mask = [self._DIO_VALVE5PORT_L]
            elif self.HAND_R == hand:
                mask = [self._DIO_VALVE5PORT_R]
        elif self.GRIPPER_TOOLCHANGE_OFF == operation:
            # TODO: Important: Need to stop the air when tool is off.
            if self.HAND_L == hand:
                # 10/29/2013 DIO changed. Now '0' is OFF for both 5PORT Valves.
                dout = mask = [self._DIO_VALVE5PORT_L]
            elif self.HAND_R == hand:
                dout = mask = [self._DIO_VALVE5PORT_R]
        elif self.GRIPPER_CLOSE == operation:
            if self.HAND_L == hand:
                dout = [self._DIO_VALVE_L_1]
        elif self.GRIPPER_OPEN == operation:
            if self.HAND_L == hand:
                dout = [self._DIO_VALVE_L_2]
        else:
            # TODO: Might want to thrown exception?
            rospy.logwarn('No gripper specified. Do nothing.')
            return
        self._dio_writer(dout, mask)

    def use_airhand(self, operation, hand=None):
        '''
        NOTE: Currently implemented for RIGHT hand only.

        @type hand: str
        @param hand: Use class member variable HAND_L or R
        '''
        dout = []
        mask = [self._DIO_EJECTOR_R_1, self._DIO_EJECTOR_R_2]

        # TODO: Implement for R hand too.
        if self.AIRHAND_DRAWIN == operation:
            if self.HAND_R == hand:
                # dout = [self._DIO_EJECTOR_R_2]  #TODO: https://bitbucket.org/tork-a/iros13/issue/37/dio#comment-6611013
                dout = [self._DIO_EJECTOR_R_1]
        elif self.AIRHAND_KEEP == operation:
            if self.HAND_R == hand:
                pass  # Do nothing since off for both pins.
        elif self.AIRHAND_RELEASE == operation:
            if self.HAND_R == hand:
                # dout = [_DIO_EJECTOR_R_1]  #TODO: https://bitbucket.org/tork-a/iros13/issue/37/dio#comment-6611013
                dout = [self._DIO_EJECTOR_R_2]

                # Create a thread to do KEEP action after the specified amount
                # of time without stopping the program.
                thread = AirhandReleaseThread(self, hand,
                                              self._SLEEP_POST_RELEASE)
                thread.start()
        else:
            # TODO: Might want to thrown exception?
            rospy.logwarn('No gripper specified. Do nothing.')
            return
        self._dio_writer(dout, mask)

    def init_dio(self):
        '''
        Initialize dio. All channels will be set '1' (off), EXCEPT for
        tool changers (channel 19 and 24) so that attached tools won't fall.
        '''
        # TODO: The behavior might not be optimized. Ask Hajime-san and
        #       Nagashima-san to take a look.

        # 10/24/2013 OUT19, 24 are alternated; When they turned to '1', they
        # are ON. So they won't fall upon this function call.

        dout = mask = []
        # Use all slots from 17 to 32.
        for i in range(16, 32):
            mask.append(i)

        self._dio_writer(dout, mask, 0)
