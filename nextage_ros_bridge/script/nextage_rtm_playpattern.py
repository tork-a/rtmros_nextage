#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Tokyo Opensource Robotics Kyokai Association
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
# Author: Yosuke Yamamoto

from nextage_ros_bridge import nextage_client
import math
###

def circularPositions(center=[0.3, 0.2, 0.1], radius=0.1 ,steps=12):
    positions_xyz = []
    step_rad = 2 * math.pi / steps
    for i in range(steps+1):
        ang_rad = step_rad * i
        px = center[0] - radius * math.cos(ang_rad)
        py = center[1] + radius * math.sin(ang_rad)
        pz = center[2]
        positions_xyz.append([px,py,pz])
    
    return positions_xyz


def rectangularPositions(dp_a=[0.25, 0.0, 0.1], dp_b=[0.45, 0.2, 0.1]):
    positions_xyz = [
                     [ dp_a[0], dp_a[1], dp_a[2] ],
                     [ dp_a[0], dp_b[1], dp_a[2] ],
                     [ dp_b[0], dp_b[1], dp_b[2] ],
                     [ dp_b[0], dp_a[1], dp_b[2] ],
                     [ dp_a[0], dp_a[1], dp_a[2] ]
                    ]
    
    return positions_xyz
    

def samePostureRPY(rpy, pat_length):
    posture_rpy = []
    for i in range(pat_length):
        posture_rpy.append(rpy)
    
    return posture_rpy


def equalTimeList(whole_tm, pat_length):
    tm_list = []
    tm = whole_tm / pat_length
    for i in range(pat_length):
        tm_list.append(tm)
    
    return tm_list


def setTargetPoseSequence(limb, pos_list, rpy_list, tm_list):
    pattern_arm = []
    for pos, rpy, tm in zip(pos_list, rpy_list, tm_list):
        robot.setTargetPose(limb, pos, rpy, tm)
        robot.waitInterpolationOfGroup(limb)
        if limb == 'rarm':
            joint_angles_deg = robot.getJointAngles()[3:9]
        else:
            joint_angles_deg = robot.getJointAngles()[9:]
        joint_angles_rad = [math.radians(angle_in_degree) for angle_in_degree in joint_angles_deg]
        pattern_arm.append(joint_angles_rad)

    return pattern_arm

###

from nextage_ros_bridge import nextage_client

if __name__ == '__main__':
    robot = nextage_client.NextageClient()
    # #robot.init(robotname=robotname, url=modelfile)
    robot.init(robotname="HiroNX(Robot)0")#, url=modelfile)

    # go initial
    #robot.goInitial()

    # playPatternOfGroup() Example
    positions_arm = []  # All elements are lists. E.g. pos1 = [x1, y1, z1]
    rpys_arm = []       # All elements are lists. E.g. rpy1 = [r1, p1, y1]
    time_list = []      # Time list [t1, t2, t3,...]
    limb_name = 'larm'  # Limb 'rarm', 'larm'

    # Rectangular Positions, RPYs, Time List
    rect_xyzs = rectangularPositions(dp_a=[0.25, 0.0, 0.1], dp_b=[0.45, 0.2, 0.1])
    rect_rpys = samePostureRPY([-3.073437, -1.569023, 3.073247], len(rect_xyzs))
    rect_time = equalTimeList(10.0, len(rect_xyzs))

    # Circular Positions, RPYs, Time List
    circ_xyzs = circularPositions(center=[0.35, 0.1, 0.1], radius=0.1 ,steps=12)
    circ_rpys = samePostureRPY([-3.073437, -1.569023, 3.073247], len(circ_xyzs))
    circ_time = equalTimeList(10.0, len(circ_xyzs))

    # Adjust Transition Time
    rect_time[0] += 1.0
    circ_time[0] += 1.0

    # Combine Patterns
    positions_arm = rect_xyzs + circ_xyzs
    rpys_arm      = rect_rpys + circ_rpys
    time_list     = rect_time + circ_time

    print('Limb: %s' % limb_name)
    print('Positions (%d): %s' % (len(positions_arm), positions_arm))
    print('RPY Pattern (%d): %s' % (len(rpys_arm), rpys_arm))
    print('Time List (%d): %s' % (len(time_list), time_list))

    # Generate Joint Angle Pattern with Moving
    print('Generating Joint Angle Pattern')
    play_pattern_arm = setTargetPoseSequence(limb_name, positions_arm, rpys_arm, time_list)
    play_pattern_time = time_list

    print('Generated')
    print('Limb: %s' % limb_name)
    print('Joint Angle Pattern [rad]: %s' % play_pattern_arm)
    print('Motion Time Pattern [sec]: %s' % play_pattern_time)

    robot.goInitial()

    # Play Pattern - playPatternOfGroup()
    print('Start: Play Pattern')
    robot.playPatternOfGroup(limb_name, play_pattern_arm, play_pattern_time)
    robot.waitInterpolationOfGroup(limb_name)
    print('End: Play Pattern')

    robot.goInitial()

