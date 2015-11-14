#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, TORK (Tokyo Opensource Robotics Kyokai Association) 
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
#  * Neither the name of Tokyo Opensource Robotics Kyokai Association
#    nor the names of its contributors may be used to endorse or promote 
#    products derived from this software without specific prior written 
#    permission.
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

# Author: Isaac I.Y. Saito

import xml.etree.ElementTree

import rospy


class MoveGroupAttr():

    def __init__(self, mg_name, joints, pose_for_test):
        '''
        @type mg_name: str
        @type joints: str[]
        @type pose_for_test: float[]  # TODO TBD
        '''
        self._mg_name = mg_name
        self._joints = joints
        self._pose_for_test = pose_for_test

    def get_joints(self):
        return self._joints

    def get_movegroup_name(self):
        return self._mg_name

    def get_pose_for_test(self):
        return self._pose_for_test


class DualArmConf():
    _movegroup_attrs = []  # MoveGroupAttr[]

    def __init__(self, dualarm_conf, srdf_xml):
        '''
        @param dualarm_conf: YAML format
        @type dualarm_conf: str
        @param srdf_xml: XML format
        @type srdf_xml: str
        @raise Exception: TBD specific type of exception
        '''
        try:
            self._movegroup_attrs = self._generate_dualarm_conf(dualarm_conf, srdf_xml)
        except Exception as e:  # TODO Better to raise more explicit exception
            rospy.logfatal('Something was wrong during config interpretation.')
            raise e

    def _generate_dualarm_conf(self, dualarm_conf, srdf_xml):
        '''
        @raise Exception: TBD specific type of exception
        '''

        mg_attrs = []  # MoveGroupAttr[]

        class YamlStruct:
            '''
            Implements http://stackoverflow.com/a/6866697/577001
            Not used this time since this doesn't allow access to lower levels in the tree hierarchy.
            '''
            def __init__(self, **entries):
                self.__dict__.update(entries)

        class YamlObj(object):
            '''Implements http://stackoverflow.com/a/1305682/577001'''
            def __init__(self, d):
                for a, b in d.items():
                    if isinstance(b, (list, tuple)):
                        setattr(self, a, [YamlObj(x) if isinstance(x, dict) else x for x in b])
                    else:
                        setattr(self, a, YamlObj(b) if isinstance(b, dict) else b)

        # Generate a yaml object for dualarm conf
        yamlobj = YamlObj(dualarm_conf)

        # Read SRDF. Since srdf here is already checked when it's obtained,
        # trust how it's obtained and no validation check is done here.
        srdf_elemtree = xml.etree.ElementTree.fromstring(srdf_xml)

        # mg names for test purpose are hardcoded here
        MOVEGROUP_NAMES_DUALARM = ['movegroup_arm_left', 'movegroup_arm_right', 'movegroup_eef_left',
                                   'movegroup_eef_right', 'movegroup_torso']

        _KEY_MOVEGROUP_NAME = 'movegroup_name'
        _KEY_MOVEGROUP_TESTPOSE_GOAL = 'test_pose_goal'
        # Seems like with YAML, control using relative names is not possible;
        # for example, in XMl you can specify root, children elements.
        #  http://stackoverflow.com/questions/4150782/using-yaml-with-variables
        #
        # That said, in the following, movegroup_arm_left looks like a variable
        # but it is actually not -- it's a constant value. Same for other elements
        # in the passed YAML file.
        mg_name = yamlobj.movegroup_arm_left.movegroup_name
        testpose_goal = yamlobj.movegroup_arm_left.test_pose_goal
        # Get movegroups from yaml, and then get corresponding values from srdf
        for elem_group in srdf_elemtree.findall('group'):
            joints = []
            if elem_group.get('name') == mg_name:  # e.g. mg_name == 'right_arm'
                for child in elem_group.getchildren():
                    if child.tag == 'joint':
                        joints.append(child.get('name'))
            mg_attrs.append(MoveGroupAttr(mg_name, joints, testpose_goal))
        return mg_attrs

    def get_movegroup_attrs(self):
        '''
        @rtype: MoveGroupAttr[]
        '''
        return self._movegroup_attrs
