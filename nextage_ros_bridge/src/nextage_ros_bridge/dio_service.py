#!/usr/bin/env python
import roslib
roslib.load_manifest('nextage_hardware_service')
import rospy

from nextage_ros_bridge import nextage_client

from hrpsys import rtm
import argparse

from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

def main():
    parser = argparse.ArgumentParser(description='ROS service interface for Nextage hardware service')
    parser.add_argument('--host', help='CORBA name server hostname')
    parser.add_argument('--port', help='CORBA name server port number')
    parser.add_argument('--modelfile', help='robot model file name')
    parser.add_argument('--robot', help='robot module name (RobotHardware0 for real robot, Robot()')
    args, unknown = parser.parse_known_args()

    if args.host:
        rtm.nshost = args.host
    if args.port:
        rtm.nsport = args.port
    if not args.robot:
        args.robot = "RobotHardware0"
    if not args.modelfile:
        args.modelfile = ""

    robot = nextage_client.NextageClient()
    robot.init(robotname=args.robot, url=args.modelfile)

    rospy.init_node("nextage_hand_dio")

    def make_on_off_service(name):
        def on_handler(req):
            getattr(robot, name)(True)
            return EmptyResponse()
        def off_handler(req):
            getattr(robot, name)(False)
            return EmptyResponse()
    	on = rospy.Service("nextage_hand_dio/" + name + "/on", Empty, on_handler)
    	off = rospy.Service("nextage_hand_dio/" + name + "/off", Empty, off_handler)
        return on, off
    make_on_off_service('turn_handlight_l')
    make_on_off_service('turn_handlight_r')
    make_on_off_service('turn_handlight_both')

    def make_empty_service(name):
        def handler(req):
            getattr(robot, name)()
            return EmptyResponse()
        return rospy.Service("nextage_hand_dio/" + name, Empty, handler)
    make_empty_service('handtool_eject_l')
    make_empty_service('handtool_eject_r')
    make_empty_service('handtool_attach_l')
    make_empty_service('handtool_attach_r')
    make_empty_service('gripper_close_l')
    make_empty_service('gripper_close_r')
    make_empty_service('gripper_open_l')
    make_empty_service('gripper_open_r')
    make_empty_service('airhand_drawin_l')
    make_empty_service('airhand_drawin_r')
    make_empty_service('airhand_keep_l')
    make_empty_service('airhand_keep_r')
    make_empty_service('airhand_release_l')
    make_empty_service('airhand_release_r')
    make_empty_service('initialize_hand_dio')

    rospy.spin()

