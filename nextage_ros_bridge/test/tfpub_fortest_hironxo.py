#!/usr/bin/env python

from hrpsys import rtm
import rospy
import tf

from nextage_ros_bridge import nextage_client


class CoordSyst():

    def __init__(self):
        rtm.nshost = 'localhost'
        rtm.nsport = 2809
        self.hiro = nextage_client.NextageClient()
#        self.hiro.init(robotname='RobotHardware0', url='/opt/jsk/etc/HIRONX/model/main.wrl')
        self.hiro.init("HiroNX(Robot)0", '')

    def publishTF(self):
        global syst
        pos = self.hiro.getCurrentPosition('LARM_JOINT5')
        pos[2] = pos[2] - self.hiro.getCurrentPosition('WAIST')[2]  # Subtract waist height
        rpy = self.hiro.getReferenceRPY('LARM_JOINT5')
        syst.sendTransform((pos[0], pos[1], pos[2]),
                           tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2]),
                           rospy.Time.now(),
                           "LARM_JOINT5_Link_TEMP",
                           "WAIST")

if __name__ == "__main__":
    rospy.init_node('tf_publisher')

    syst = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    coordSyst = CoordSyst()

    while not rospy.is_shutdown():
        coordSyst.publishTF()
        rate.sleep()
