#!/usr/bin/env python

import rospy
import geometry_msgs.msg

from moveit_commander import MoveGroupCommander

if __name__ == '__main__':
    rospy.init_node('commander_example', anonymous=True)
    group = MoveGroupCommander("right_arm")
  
    # move to a random target
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.y = 1.0
    pose_target.position.x =  0.8
    pose_target.position.y = -0.2
    pose_target.position.z =  0.0
    rospy.loginfo("set target to {}".format(pose_target))
    group.set_pose_target(pose_target)
    plan = group.plan()
    rospy.loginfo("plan is {}".format(plan))
    ret = group.go()
    rospy.loginfo("executed ... {}".format(ret))

    pose_target.position.x =  0.5
    pose_target.position.y = -0.9
    rospy.loginfo("set target to {}".format(pose_target))
    group.set_pose_target(pose_target)
    plan = group.plan()
    rospy.loginfo("plan is {}".format(plan))
    ret = group.go()
    rospy.loginfo("executed ... {}".format(ret))
