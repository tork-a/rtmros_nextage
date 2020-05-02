#!/usr/bin/env python  

import rospy
import tf
import geometry_msgs.msg
import sys

from moveit_commander import MoveGroupCommander
from tf.transformations import *
import math

def kbhit():
    import select
    return sys.stdin in select.select([sys.stdin], [], [], 0)[0]

if __name__ == '__main__':
    rospy.init_node('ar_pose_commander', anonymous=True)
    group = MoveGroupCommander("right_arm")
    
    base_frame_id = '/WAIST'
    ar_marker_id = '/ar_marker_4'
    
    pub = rospy.Publisher('target_pose', geometry_msgs.msg.PoseStamped)
    listener = tf.TransformListener()
    
    rate = rospy.Rate(10.0)
    pose_st_target = None
    while not rospy.is_shutdown() and not kbhit():
        try:
            now = rospy.Time(0)
            (trans,quat) = listener.lookupTransform(base_frame_id, ar_marker_id, now)
            quat = quaternion_multiply(quat, quaternion_about_axis(math.pi/2, (1,0,0)))
            quat = quaternion_multiply(quat, quaternion_about_axis(math.pi/2, (0,0,1)))
            pose_st_target = geometry_msgs.msg.PoseStamped()
            pose_st_target.pose.position.x = trans[0]
            pose_st_target.pose.position.y = trans[1]
            pose_st_target.pose.position.z = trans[2]
            pose_st_target.pose.orientation.x = quat[0]
            pose_st_target.pose.orientation.y = quat[1]
            pose_st_target.pose.orientation.z = quat[2]
            pose_st_target.pose.orientation.w = quat[3]
            pose_st_target.header.frame_id = base_frame_id
            pose_st_target.header.stamp = now
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(e)

        if pose_st_target:
            pub.publish(pose_st_target)
            rospy.loginfo(trans)

        rate.sleep()

    if raw_input() == 'q':
        sys.exit(1)

    # move to a point above ar marker
    pose_st_target.pose.position.z += 0.3
    rospy.loginfo("set target to {}".format(pose_st_target.pose))
    group.set_pose_target(pose_st_target.pose)
    plan = group.plan()
    rospy.loginfo("plan is {}".format(plan))
    ret = group.go()
    rospy.loginfo("executed ... {}".format(ret))

