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

import rospy
import geometry_msgs.msg
import copy
import tf

from moveit_commander import MoveGroupCommander

import nextage_rtm_playpattern as nxtpp


def setTargetPoseSequenceMoveIt(limb, pos_list, rpy_list, tm_list):
    '''
    Create a array of limb joint angles for the each waypoints
    from data of the end effector postions and postures
    using MoveIt!.
    
    @type  limb     : str
    @param limb     : limb to create a pattern, right arm 'rarm' or left arm 'larm'
    @type  pos_list : [[float,float,float],[float,float,float],...]
    @param pos_list : Array of end effector positions (x,y,z) [m]
    @type  rpy_list : [[float,float,float],[float,float,float],...]
    @param rpy_list : Array of end effector postures (r,p,y) [m]
    @type  tm_list  : [float,float,...]
    @param tm_list  : Array of motion time lengths of the each pose [s]
    
    @rtype  : RobotTrajectory, float
    @return : Plan of limb waypoints motion and fraction of the path achieved as described by the waypoints
    '''
    wpose = geometry_msgs.msg.Pose()
    waypoints = []
    
    for pos, rpy, tm in zip(pos_list, rpy_list, tm_list):
        quaternion = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        wpose.position.x = pos[0]
        wpose.position.y = pos[1]
        wpose.position.z = pos[2]
        wpose.orientation.x = quaternion[0]
        wpose.orientation.y = quaternion[1]
        wpose.orientation.z = quaternion[2]
        wpose.orientation.w = quaternion[3]
        waypoints.append(copy.deepcopy(wpose))
    
    (pln, frc) = limb.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold
    
    return pln, frc


if __name__ == '__main__':
    '''
    Main sequence for moving at waypoints
    using MoveIt!.
    '''
    rospy.init_node('commander_example', anonymous=True)

    limb_name = "left_arm"  # Limb 'right_arm', 'left_arm'
    group = MoveGroupCommander(limb_name)
  
    rarm_initial_pose = group.get_current_pose().pose
    rospy.loginfo("Got Initial Pose \n{}".format(rarm_initial_pose))
    
    pose_target = geometry_msgs.msg.Pose()
    pose_target = copy.deepcopy(rarm_initial_pose)
    
    positions_arm = []  # All elements are lists. E.g. pos1 = [x1, y1, z1]
    rpys_arm = []       # All elements are lists. E.g. rpy1 = [r1, p1, y1]
    time_list = []      # Time list [t1, t2, t3,...]
    
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
    
    # plan waypoints
    rospy.loginfo("Planing Waypoints")
    plan, flaction = setTargetPoseSequenceMoveIt(group, positions_arm, rpys_arm, time_list)
    
    # wait for playing the plan on MoveIt
    rospy.loginfo("Waiting while the plan is visualized on RViz")
    rospy.sleep(20)
    
    # execute the plan
    rospy.loginfo("Execute!!")
    group.execute(plan)
    rospy.loginfo("Executiion Done")
    rospy.sleep(5)

    # move to the initial pose
    rospy.loginfo("Move to Initial Pose")
    pose_target = copy.deepcopy(rarm_initial_pose)
    rospy.loginfo("set target to {}".format(pose_target))
    group.set_pose_target(pose_target)
    plan = group.plan()
    ret = group.go()
