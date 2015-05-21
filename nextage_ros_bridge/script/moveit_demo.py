#!/usr/bin/env python

import rospy
import geometry_msgs.msg

from moveit_commander import MoveGroupCommander

if __name__ == '__main__':
    rospy.init_node('commander_example', anonymous=True)
    group = MoveGroupCommander("right_arm")
  
    # move to a random target
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.y = 0.003
    #pose_target.position.x =  0.1
    #pose_target.position.y = -0.2
    #pose_target.position.z =  -0.05
    pose_target.position.x = 0.2035
    pose_target.position.y = -0.5399
    pose_target.position.z = 0.0709
    #pose_target.orientation.x = 0.000427
    #pose_target.orientation.y = 0.000317
    #pose_target.orientation.z = -0.000384
    pose_target.orientation.w = 0.999999
    rospy.loginfo("set target to {}".format(pose_target))
    group.set_pose_target(pose_target)
    plan = group.plan()
    rospy.loginfo("plan is {}".format(plan))
    ret = group.go()
    rospy.loginfo("executed ... {}".format(ret))

    pose_target.position.x =  0.05
    pose_target.position.y = -0.09
    rospy.loginfo("set target to {}".format(pose_target))
    group.set_pose_target(pose_target)
    plan = group.plan()
    rospy.loginfo("plan is {}".format(plan))
    ret = group.go()
    rospy.loginfo("executed ... {}".format(ret))
