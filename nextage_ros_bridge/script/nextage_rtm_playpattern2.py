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
import rospy
import geometry_msgs.msg
import copy, tf, math

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest

import nextage_rtm_playpattern as nxtpp


def setTargetPoseSequenceRTM(limb, pos_list, rpy_list, tm_list):
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

def setTargetPoseSequenceMoveIt(limb, pos_list, rpy_list, tm_list):
    '''
    Create a array of limb joint angles for the each waypoints
    from data of the end effector postions and postures
    using MoveIt! IK for computing joint angles.
    
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
    wpose = geometry_msgs.msg.Pose()

    rospy.wait_for_service('compute_ik')
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    if limb == 'rarm':
        limb_name = "right_arm"  # Limb 'right_arm', 'left_arm'
    else:
        limb_name = "left_arm"

    joint_name = filter(lambda n:n[0] == limb, robot.Groups)[0][1]

    for pos, rpy, tm in zip(pos_list, rpy_list, tm_list):
        quaternion = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        wpose.position.x = pos[0]
        wpose.position.y = pos[1]
        wpose.position.z = pos[2]
        wpose.orientation.x = quaternion[0]
        wpose.orientation.y = quaternion[1]
        wpose.orientation.z = quaternion[2]
        wpose.orientation.w = quaternion[3]
        req = PositionIKRequest()
        req.group_name = limb_name
        #req.ik_link_name = ''
        req.pose_stamped.pose = wpose
        ret = compute_ik(req)
        if ret.error_code.val != 1:
            error(ret)

        pattern_arm.append(map(lambda n: n[1],filter(lambda n:n[0] in joint_name, zip(ret.solution.joint_state.name, ret.solution.joint_state.position))))
    return pattern_arm

if __name__ == '__main__':
    '''
    Main sequence for moving at waypoints with RTM interface
    using MoveIt! IK for computing joint angles.
    '''
    robot = nextage_client.NextageClient()
    # #robot.init(robotname=robotname, url=modelfile)
    robot.init(robotname="HiroNX(Robot)0")#, url=modelfile)

    # go initial
    #robot.goInitial()
    
    positions_arm = []  # All elements are lists. E.g. pos1 = [x1, y1, z1]
    rpys_arm = []       # All elements are lists. E.g. rpy1 = [r1, p1, y1]
    time_list = []      # Time list [t1, t2, t3,...]
    limb_name = 'larm'  # Limb 'rarm', 'larm'
    
    # Rectangular Positions, RPYs, Time List
    rect_xyzs = nxtpp.rectangularPositions(dp_a=[0.25, 0.0, 0.1], dp_b=[0.45, 0.2, 0.1])
    rect_rpys = nxtpp.samePostureRPY([-3.073437, -1.569023, 3.073247], len(rect_xyzs))
    rect_time = nxtpp.equalTimeList(10.0, len(rect_xyzs))
    
    # Circular Positions, RPYs, Time List
    circ_xyzs = nxtpp.circularPositions(center=[0.35, 0.1, 0.1], radius=0.1 ,steps=12)
    circ_rpys = nxtpp.samePostureRPY([-3.073437, -1.569023, 3.073247], len(circ_xyzs))
    circ_time = nxtpp.equalTimeList(10.0, len(circ_xyzs))

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
    
    # Generate Joint Angle Pattern with MoveIt IK
    print('Generating Joint Angle Pattern')
    play_pattern_arm = setTargetPoseSequenceMoveIt(limb_name, positions_arm, rpys_arm, time_list)
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
