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
    '''
    Create a array of circular position coordinates
    from a center position and a radius divided by steps.
    
    @type  center : [float, float, float]
    @param center : Position (x,y,z) [m] coordinates of a circle center
    @type  radius : float
    @param radius : Radius length [m]
    @type  steps  : int
    @param steps  : Number of steps for dividing a circle
    
    @rtype  : [ [float, float, float], [float, float, float], ...]
    @return : Array of circular position coordinates (x,y,z) [m]
              Length of steps+1 (including an end position same as the start position)
    '''
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
    '''
    Create a array of rectangular position coordinates
    from diagonal points of A and B.
    
    @type  dp_a : [float, float, float]
    @param dp_a : Position (x,y,z) [m] coordinates of a diagonal point A
    @type  dp_a : [float, float, float]
    @param dp_a : Position (x,y,z) [m] coordinates of a diagonal point B
    
    @rtype  : [ [float, float, float], [float, float, float], ...]
    @return : Array of rectangular position coordinates (x,y,z) [m]
              Length of 5 (including an end position same as the start position)
    '''
    positions_xyz = [
                     [ dp_a[0], dp_a[1], dp_a[2] ],
                     [ dp_a[0], dp_b[1], dp_a[2] ],
                     [ dp_b[0], dp_b[1], dp_b[2] ],
                     [ dp_b[0], dp_a[1], dp_b[2] ],
                     [ dp_a[0], dp_a[1], dp_a[2] ]
                    ]
    
    return positions_xyz
    

def samePostureRPY(rpy, pat_length):
    '''
    Create a array of the same posture Roll, Pitch, Yaw angles
    from one set of Roll Pitch Yaw angles.
    
    @type  rpy        : [float, float, float]
    @param rpy        : Posture angle Roll, Pitch, Yaw (r,p,y) [rad] to copy
    @type  pat_length : int
    @param pat_length : Array length of pat_length for the same posture angles
    
    @rtype  : [ [float, float, float], [float, float, float], ...]
    @return : Array of the same posture angles (r,p,y) [rad]
              Length of pat_length
    '''
    posture_rpy = []
    for i in range(pat_length):
        posture_rpy.append(rpy)
    
    return posture_rpy


def equalTimeList(whole_tm, pat_length):
    '''
    Create a array of the same time length
    from whole motion time.
    
    @type  whole_tm   : float
    @param whole_tm   : Whole time [s]
    @type  pat_length : int
    @param pat_length : Number to divide the whole time
    
    @rtype  : [ float, float, float, ... ]
    @return : Array of the equally divided whole time [s]
              Length of pat_length
    '''
    tm_list = []
    tm = whole_tm / pat_length
    for i in range(pat_length):
        tm_list.append(tm)
    
    return tm_list


def setTargetPoseSequence(limb, pos_list, rpy_list, tm_list):
    '''
    Create a array of limb joint angles for the each waypoints
    from data of the end effector postions and postures
    using robot moving using RTM interface.
    
    @type  limb     : str
    @param limb     : limb to create a pattern, right arm 'rarm' or left arm 'larm'
    @type  pos_list : [[float,float,float],[float,float,float],...]
    @param pos_list : Array of end effector positions (x,y,z) [m]
    @type  rpy_list : [[float,float,float],[float,float,float],...]
    @param rpy_list : Array of end effector postures (r,p,y) [m]
    @type  tm_list  : [float,float,...]
    @param tm_list  : Array of motion time lengths of the each pose [s]
    
    @rtype  : [[float,float,...],[float,float,...],...]
    @return : Array of limb joint angles [rad]
    '''
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
    '''
    Main sequence for moving at waypoints
    using RTM interface.
    '''
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

