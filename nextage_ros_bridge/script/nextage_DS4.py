#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

from hironx_ros_bridge.ros_client import ROS_Client
# This should come earlier than later import.
# See http://code.google.com/p/rtm-ros-robotics/source/detail?r=6773
from nextage_ros_bridge import nextage_client

from hrpsys import rtm
import argparse

#------------------my definition callback function-------------------
import rospy

def callback(data):
#-----------move head of YOW angle-----------------
    #-----------circle bottun-----------------------
    if data.buttons[2] == 1 and data.buttons[6] == 1 and data.buttons[7] != 1:
        robot.setTargetPoseRelative('head', 'HEAD_JOINT1', dw=0.01, tm=0.001, wait=False)
    #-----------squire bottun----------------------
    elif data.buttons[0] == 1 and data.buttons[6] == 1 and data.buttons[7] != 1:
        robot.setTargetPoseRelative('head', 'HEAD_JOINT1', dw=-0.01, tm=0.001, wait=False)
#--------------------------------------------------


#-----------move head of PITCH angle-----------------
    #-----------circle bottun-----------------------
    if data.buttons[2] == 1 and data.buttons[6] == 1 and data.buttons[7] == 1:
        robot.setTargetPoseRelative('head', 'HEAD_JOINT1', dp=0.01, tm=0.001, wait=False)
    #-----------squire bottun----------------------
    elif data.buttons[0] == 1 and data.buttons[6] == 1 and data.buttons[7] == 1:
        robot.setTargetPoseRelative('head', 'HEAD_JOINT1', dp=-0.01, tm=0.001, wait=False)
#--------------------------------------------------


#----------Initial and OFF pose--------------------
    #-----------x bottun---------------------------
    if data.buttons[1] == 1:
        robot.goInitial(tm=1, wait=False)
    #-----------triangle bottun-------------------
    elif data.buttons[3] == 1:
        robot.goOffPose(tm=1)
#--------------------------------------------------

        
#---------------move hand control------------------
    #-----------right hand control-----------------
    if data.axes[0] != 0 and data.buttons[6] == 1 and data.buttons[4] != 1:
        robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dy=-data.axes[0]*0.01, tm=0.001, wait=False)
    elif data.axes[1] != 0 and data.buttons[6] == 1 and data.buttons[4] != 1:
        robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dx=-data.axes[1]*0.01, tm=0.001, wait=False)
    elif data.axes[1] != 0 and data.buttons[6] == 1 and data.buttons[4] == 1:
        robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dz=data.axes[1]*0.01, tm=0.001, wait=False)
    #-----------left hand control-------------
    if data.axes[2] != 0 and data.buttons[6] == 1 and data.buttons[4] != 1:
        robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dy=-data.axes[2]*0.01, tm=0.001, wait=False)
    elif data.axes[5] != 0 and data.buttons[6] == 1 and data.buttons[4] != 1:
        robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dx=-data.axes[5]*0.01, tm=0.001, wait=False)
    elif data.axes[5] != 0 and data.buttons[6] == 1 and data.buttons[4] == 1:
        robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dz=data.axes[5]*0.01, tm=0.001, wait=False)
#--------------------------------------------------


    #rospy.loginfo(data.buttons[3])

    #rospy.loginfo(data.axes[0])
#--------------------------------------------------------------------


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='hiro command line interpreters')
    parser.add_argument('--host', help='corba name server hostname')
    parser.add_argument('--port', help='corba name server port number')
    parser.add_argument('--modelfile', help='robot model file nmae')
    parser.add_argument('--robot', help='robot modlule name (RobotHardware0 for real robot, Robot()')
    args, unknown = parser.parse_known_args()

    if args.host:
        rtm.nshost = args.host
    if args.port:
        rtm.nsport = args.port
    if not args.robot:
        args.robot = "RobotHardware0" if args.host else "HiroNX(Robot)0"
    if not args.modelfile:
        args.modelfile = ""

    # support old style format
    if len(unknown) >= 2:
        args.robot = unknown[0]
        args.modelfile = unknown[1]
    robot = nxc = nextage_client.NextageClient()
    # Use generic name for the robot instance. This enables users on the
    # script commandline (eg. ipython) to run the same commands without asking
    # them to specifically tell what robot they're using (eg. hiro, nxc).
    # This is backward compatible so that users can still keep using `nxc`.
    # See http://code.google.com/p/rtm-ros-robotics/source/detail?r=6926
    robot.init(robotname=args.robot, url=args.modelfile)

    # ROS Client.
    ros = ROS_Client()

# for simulated robot
# $ ./hironx.py
#
# for real robot
# ./hironx.py  --host hiro014
# ./ipython -i hironx.py --host hiro014
# for real robot with custom model file
# ./hironx.py  --host hiro014 --modelfile /opt/jsk/etc/NEXTAGE/model/main.wrl


#--------------------here is my definition fucntion-----------------------------
import math
from sensor_msgs.msg import Joy

print("-----------------hello----------------------")
rospy.Subscriber("joy", Joy, callback)
rospy.spin()
#------------------------------------------------------------------------------

