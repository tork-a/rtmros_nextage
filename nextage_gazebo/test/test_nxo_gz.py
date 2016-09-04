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

import unittest

from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
import rospy


class TestNxoGazebo(unittest.TestCase):
    LINKS = ['ground_plane::link', 'NextageOpen::WAIST', 
             'NextageOpen::CHEST_JOINT0_Link',
             'NextageOpen::HEAD_JOINT0_Link', 'NextageOpen::HEAD_JOINT1_Link',
             'NextageOpen::LARM_JOINT0_Link', 'NextageOpen::LARM_JOINT1_Link', 'NextageOpen::LARM_JOINT2_Link', 'NextageOpen::LARM_JOINT3_Link', 'NextageOpen::LARM_JOINT4_Link', 'NextageOpen::LARM_JOINT5_Link',
             'NextageOpen::RARM_JOINT0_Link', 'NextageOpen::RARM_JOINT1_Link', 'NextageOpen::RARM_JOINT2_Link', 'NextageOpen::RARM_JOINT3_Link', 'NextageOpen::RARM_JOINT4_Link', 'NextageOpen::RARM_JOINT5_Link']
    POSES_INIT = [
                  [[ 0.0,  0.0,  0.0], [ 0.0,  0.0,  0.0,  1.0]],
                  [[ 0.00869031777881, 0.00936108107894,  0.970000019992], [ 4.52986285804e-06,  9.1893585999e-07,  0.000198866987408,  0.999999980215]],
                  [[ 0.00869026831415, 0.00936080520858,  0.969999965706], [ 2.94622369018e-06,  1.3456210695e-06,  -0.000126180352162,  0.999999992034]],
                  [[ 0.00869141396909, 0.00935955870271,  1.53949992447], [ 8.45267344191e-07,  7.30178323418e-07,  -0.000131509094738,  0.999999991352]],
                  [[ 0.00869143715228, 0.0093595715095,  1.53949989454], [ 1.01216879393e-06,  4.83962982629e-06,  -0.000131441661779,  0.999999991349]],
                  [[ 0.00872762107774, 0.154360006834,  1.34029621598], [ -0.130527499716,  3.7821173448e-05,  2.6239331832e-05,  0.991444688169]],
                  [[ 0.00872766863833, 0.154360191609,  1.34029623983], [ -0.130527069459,  0.000269559123027,  -4.63162615923e-06,  0.991444709227]],
                  [[ 0.00858791298884, 0.181417572639,  1.07422723747], [ -0.091005332606,  -0.710566966204,  0.0935703234198,  0.691416813906]],
                  [[ 0.00940755565144, 0.173655626629,  1.04526064458], [ -0.090885098394,  -0.710582204533,  0.0934535149189,  0.691432766103]],
                  [[ 0.244320109519, 0.175331850808,  1.05145851947], [ -0.0909400891469,  -0.710174638341,  0.0934000106546,  0.691851372228]],
                  [[ 0.335517794548, 0.163801033044,  1.00834272469], [ -0.0908880315601,  -0.710167606022,  0.0934534767984,  0.69185821145]],
                  [[ 0.00865434250886, -0.135639383827,  1.34029577734], [ 0.13052792502,  9.10844525273e-06,  -0.000308631661141,  0.991444585165]],
                  [[ 0.00865441900623, -0.135638679716,  1.34029592469], [ 0.130523056036,  0.000264943619984,  -0.000273859879479,  0.991445201032]],
                  [[ 0.00848275607192, -0.162697817784,  1.07422672949], [ 0.0908048466902,  -0.71057338265,  -0.093765722566,  0.691410107649]],
                  [[ 0.0093050984273, -0.154936315414,  1.04526041866], [ 0.0907054333018,  -0.710585857087,  -0.0936678744598,  0.691423599085]],
                  [[ 0.244217095143, -0.156743001062,  1.05144546797], [ 0.0907579071769,  -0.710197835911,  -0.0936164077221,  0.691822234651]],
                  [[ 0.335421275762, -0.145261511825,  1.0083303359], [ 0.0907167631829,  -0.710192142524,  -0.0936591102354,  0.691827695777]]]

    def _cb_gz_linkstates(self, data):
        self._linkstates = data

    def __init__(self, *args, **kwargs):
        super(TestNxoGazebo, self).__init__(*args, **kwargs)
        rospy.init_node('test_nxo_gazebo')
        rospy.loginfo("need to wait for finishing go_initial.py (https://github.com/tork-a/rtmros_nextage/pull/223/files#diff-16b25951a50b1e80569929d32a09102bR14)")
        rospy.sleep(3+4+3)
        rospy.sleep(5) # make sure robot stops
        rospy.loginfo("start test")
        self._subscriber_gz_linkstates = rospy.Subscriber('/gazebo/link_states', LinkStates, self._cb_gz_linkstates)

    @classmethod
    def setUpClass(self):
        rospy.sleep(5)  # intentionally wait for nextage_gazebo.go_initial to be done.

    @classmethod
    def tearDownClass(self):
        True  # TODO impl something meaningful

    def test_go_initial(self):
        '''Check if arms are moved to init pose'''
        _ORDER_PERMISSIBLE = 1
        # Assert list of the links
        self.assertEqual(sorted(self.LINKS), sorted(self._linkstates.name))

        # Assert if joint values are approximate?
        i = 0
        for pose in self._linkstates.pose:
            rospy.loginfo("check pose " + self.LINKS[i])
            # for coord in pose.position:  # pose.position is an instance of `list`
            # For some reasons, `for coord in pose.position` yields error `'Point' object is not iterable`
            self.assertAlmostEqual(self.POSES_INIT[i][0][0], pose.position.x, places = _ORDER_PERMISSIBLE)
            self.assertAlmostEqual(self.POSES_INIT[i][0][1], pose.position.y, places = _ORDER_PERMISSIBLE)
            self.assertAlmostEqual(self.POSES_INIT[i][0][2], pose.position.z, places = _ORDER_PERMISSIBLE)
            self.assertAlmostEqual(self.POSES_INIT[i][1][0], pose.orientation.x, places = _ORDER_PERMISSIBLE)
            self.assertAlmostEqual(self.POSES_INIT[i][1][1], pose.orientation.y, places = _ORDER_PERMISSIBLE)
            self.assertAlmostEqual(self.POSES_INIT[i][1][2], pose.orientation.z, places = _ORDER_PERMISSIBLE)
            self.assertAlmostEqual(self.POSES_INIT[i][1][3], pose.orientation.w, places = _ORDER_PERMISSIBLE)
            i += 1

if __name__ == '__main__':
    import rostest
    rostest.rosrun('nextage_gazebo', 'test_nxo_gz', TestNxoGazebo)
