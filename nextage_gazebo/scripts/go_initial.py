#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Tokyo Opensource Robotics Kyokai Association (TORK)
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

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

rospy.init_node('send_motion')
r_client = actionlib.SimpleActionClient('/rarm_controller/follow_joint_trajectory_action', FollowJointTrajectoryAction)
l_client = actionlib.SimpleActionClient('/larm_controller/follow_joint_trajectory_action', FollowJointTrajectoryAction)
h_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory_action', FollowJointTrajectoryAction)
c_client = actionlib.SimpleActionClient('/torso_controller/follow_joint_trajectory_action', FollowJointTrajectoryAction)
r_client.wait_for_server()
l_client.wait_for_server()
h_client.wait_for_server()
c_client.wait_for_server()

# gen msg
r_msg = FollowJointTrajectoryGoal()
r_msg.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
r_msg.trajectory.joint_names = ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5']
r_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0,0,-2.1,0,0,0], time_from_start = rospy.Duration(0.1)))
r_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0,0,-2.1,0,0,0], time_from_start = rospy.Duration(2)))
r_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0,0,-1.6,0,0,0], time_from_start = rospy.Duration(3)))

l_msg = FollowJointTrajectoryGoal()
l_msg.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
l_msg.trajectory.joint_names = ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5']
l_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0,0,-2.1,0,0,0], time_from_start = rospy.Duration(0.1)))
l_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0,0,-2.1,0,0,0], time_from_start = rospy.Duration(2)))
l_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0,0,-1.6,0,0,0], time_from_start = rospy.Duration(3)))

# send to robot arm
r_client.send_goal(r_msg)
l_client.send_goal(l_msg)

r_client.wait_for_result()
l_client.wait_for_result()

# head
h_msg = FollowJointTrajectoryGoal()
h_msg.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
h_msg.trajectory.joint_names = ['HEAD_JOINT0', 'HEAD_JOINT1']
h_msg.trajectory.points.append(JointTrajectoryPoint(positions=[ 1.0, 0.0], time_from_start = rospy.Duration(1)))
h_msg.trajectory.points.append(JointTrajectoryPoint(positions=[-1.0,-0.2], time_from_start = rospy.Duration(2)))
h_msg.trajectory.points.append(JointTrajectoryPoint(positions=[ 0.0, 0.2], time_from_start = rospy.Duration(3)))
h_msg.trajectory.points.append(JointTrajectoryPoint(positions=[ 0.0, 0.0], time_from_start = rospy.Duration(4)))

h_client.send_goal(h_msg)
h_client.wait_for_result()

# chest
c_msg = FollowJointTrajectoryGoal()
c_msg.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
c_msg.trajectory.joint_names = ['CHEST_JOINT0']
c_msg.trajectory.points.append(JointTrajectoryPoint(positions=[ 1.0], time_from_start = rospy.Duration(1)))
c_msg.trajectory.points.append(JointTrajectoryPoint(positions=[-1.0], time_from_start = rospy.Duration(2)))
c_msg.trajectory.points.append(JointTrajectoryPoint(positions=[ 0.0], time_from_start = rospy.Duration(3)))

c_client.send_goal(c_msg)
c_client.wait_for_result()

rospy.loginfo("done")


