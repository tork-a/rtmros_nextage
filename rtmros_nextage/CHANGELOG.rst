^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtmros_nextage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.14 (2017-02-21)
-------------------
* [capability] hand camera now available on hrpsys simulator.
* Contributors: Kei Okada, Isaac I.Y. Saito

0.7.13 (2017-01-24)
-------------------
* [fix] Left arm position issue with IKFast `#277 <https://github.com/tork-a/rtmros_nextage/issues/277>`_
* [capability] Add config for trac_ik.
* Contributors: Kei Okada, Isaac I.Y. Saito

0.7.12 (2017-01-10)
-------------------
* [fix][nxo client] Fix wrong string print that could stop the script to work. `#276 <https://github.com/tork-a/rtmros_nextage/pull/276>`_
* [capability] Release nextage_ik_plugin `#281 <https://github.com/tork-a/rtmros_nextage/pull/281>`_
* [enhance] Add simple rviz.launch that can be run without extra processes (e.g. moveit) to be running. `#282 <https://github.com/tork-a/rtmros_nextage/pull/282>`_
* [enhance][nextage.py script] users with older dio spec robot can specify dio version upon starting the script (enhance for `#274 <https://github.com/tork-a/rtmros_nextage/issues/274>`_).
* [maintenance] clean up for catkin lint `#283 <https://github.com/tork-a/rtmros_nextage/pull/283>`_
* [maintenance][fix][test] Correct api version number for dio.
* Contributors: Kei Okada, Isaac I.Y. Saito

0.7.11 (2016-11-05)
-------------------
* [improve][nextage.py] Setting default model file location for the real robot so that you'll never need to pass arg from commandline.
* [improve] Add convenient launch file for NXO with hrpsys older than 315.2.7 (related: https://github.com/tork-a/rtmros_nextage/issues/153).
* Contributors: Isaac I.Y. Saito

0.7.10 (2016-10-19)
-------------------
* [improve] getRTCList now inherits the behavior from the super class, HIRONX, to avoid duplicate.
* [improve] moderate Interactive Marker size.
* Contributors: Isaac I.Y. Saito

0.7.9 (2016-10-13)
------------------
* [fix][nxo client] `#262 <https://github.com/tork-a/rtmros_nextage/issues/262>`_ by catching error when ros master is not running
* [fix] nextage_clinet with gazebo under no rtm environment.
* [fix] test_moveit.py is very unstalble, run go 3 times
* [fix] NextageROS.desktop.in : now rtmlaunch is installed in global bin
* [fix][test] Better utilization of tests. test/test_moveit.py : botharms.go is very unstable
* [fix][joy.test] fix again, it seems they did not publish joint state publisher, so question is why it passed the test?
* [fix] remove torso from group, that outputs "Group 'torso' is not a chain" ERROR
* [fix] kinematics_ikfast.yaml : no ik fast plugin for head and torso
* [fix][test] gz.test must be one file to avoid collsion on gazebo master uri
* [fix] Missing installation
* [fix] : test/test_nxo_gz.py, because of changeing contorller parameter?
* [fix] do not have to fix /WAIST to /world, if you want run static_transform_publisher, see `#235 <https://github.com/tork-a/rtmros_nextage/issues/235>`_
* [capability] Activate ImpedanceController RTC. `#261 <https://github.com/tork-a/rtmros_nextage/issues/261>`_
* [improve] better initial hrpsys view
* [improve] tune pid parameter for gazebo
* [improve] set ARM_JOINT0 max effor to 150, ARM_JOINT1 max effor to 200, may be this is larget than real robot
* [improve] set mass of base link to super heavy
* Contributors: Kei Okada, Isaac I.Y. Saito

0.7.8 (2016-07-01)
------------------
* [feat] Add joystick launch (https://groups.google.com/d/msg/moveit-users/lbUwNqiMuP8/DZX2Fn0EAQAJ).
* Contributors: Kei Okada, Isaac I.Y. Saito

0.7.7 (2016-05-16)
------------------
* [fix] Wrong collada file location for real.launch.
* [fix] Use NXO file instead of Hironx as a collision detector input.
* Contributors: Isaac I.Y. Saito

0.7.6 (2016-02-09)
------------------
* [fix] Use RRT as default planner (workaround/fix `#170 <https://github.com/tork-a/rtmros_nextage/issues/170>`_)
* [improve] better view for hrpsys-simulator
* Contributors: Kei Okada, Isaac I.Y. Saito

0.7.5 (2016-01-27)
------------------
* [fix] fix namespace so that existing app can operate the robot on Gazebo
* Contributors: Kei Okada

0.7.4 (2016-01-26)
------------------
* [gazebo] Add non-zero mass so that links are not ignored
* Contributors: Isaac I.Y. Saito

0.7.3 (2015-12-31)
------------------
* [fix] Unnecessarily complicated dependency
* [test] Add simple tests for MoveIt
* [sys] clean up .launch and .test files
* [doc] Fix wrong URL
* Contributors: Kei Okada, Isaac I.Y. Saito

0.7.2 (2015-11-02)
------------------
* [fix] Manually move ikfast.h to package specific location
* Contributors: Kei Okada

0.7.1 (2015-10-26)
------------------
* [feat] Add torso to botharm Move Group (address `#198 <https://github.com/tork-a/rtmros_nextage/issues/198>`_)
* Contributors: Isaac I.Y. Saito

0.6.6 (2015-10-17)
------------------
* [feat] Add torso, head, and (whole) upperbody Move Group
* [fix] Correct param file names for OMPL
* Contributors: Isaac I.Y. Saito

0.6.5 (2015-10-16)
------------------
* [Fix] stop air blows out when tools get ejected (fix `#158 <https://github.com/tork-a/rtmros_nextage/issues/158>`_)
* [Feat] Print reason why the command call errors.
* [Fix] fix ar.launch
* Contributors: Isaac IY Saito, Yutaka Kondo

0.6.4 (2015-10-02)
------------------

0.6.3 (2015-08-15)
------------------
* [feat] Add launch file for AR alvar 
* [feat] Add `botharms`' MoveIt! group.
* [feat] Add IKFast plugin. Kinematics solver is now selectable in Moveit launch
* Contributors: TORK 534o, Ryu Yamamoto, Isaac IY Saito

0.6.2 (2015-05-12)
------------------
* (Doc) Correct changelogs to apply the important announcement.
* (Feature) Add hands_ueye.launch for bringing up hand's ueye camera nodes.
* (Fix) [test_handlight.py] fix to pass the test, handlight (writeDigitalOutput always returns True in simulation https://github.com/fkanehiro/hrpsys-base/blob/master/python/hrpsys_config.py#L1284)
* Contributors: Kei Okada, Ryosuke Tajima, Isaac IY Saito

0.6.1 (2015-03-09)
------------------
* Start ROS clinent when the script begins
* Contributors: Kei Okada

0.6.0 (2015-02-03)
------------------
* (IMPORTANT) VRML stored location inside qnx has now become NEXTAGE specific. Please see https://github.com/tork-a/rtmros_nextage/issues/153 for possible required actions.
* Adjust to catkin build 
* Contributors: Isaac IY Saito, Ryohei Ueda

0.5.3 (2014-11-13)
------------------
* (DIO) Fix `#129 <https://github.com/tork-a/rtmros_nextage/issues/129>`_
* (doc) Move tutorial wiki backup to hironx pkg.
* Move a python module to call DIO via rosservice.
* Contributors: Isaac IY Saito, Akio Ochiai, Daiki Maekawa

0.5.2 (2014-11-03)
------------------
* Improvement on camera launch file (add binning arg in nextage_ueye_stereo.launch)
* DIO 

 * Add DIO pin config for Aug 2014 version. Flexibly configurable DIO pin. Fix `#113 <https://github.com/tork-a/rtmros_nextage/issues/113>`_
 * Fix wrong test assertion (fix `#116 <https://github.com/tork-a/rtmros_nextage/issues/116>`_).
 * Add to test cases the check for old spec. Hand05 class returns DIO commands' results.
* Contributors: Isaac IY Saito, Yutaka Kondo

0.5.1 (2014-10-17)
------------------
* Increment minor version to 0.5, due to DIO spec update.
* Add DIO pin config for version 0.5 (Aug 2014. Fix `#113 <https://github.com/tork-a/rtmros_nextage/issues/113>`_)
* Flexibly configurable DIO pin assignment. 
* Contributors: Isaac IY Saito

0.4.2 (2014-10-01)
------------------
* Add a launch file for stereo camera (ueye).
* (Airhand) Fix wrong dio pin set for left airhand.
* Contributors: Kei Okada, Isaac IY Saito

0.4.1 (2014-09-03)
------------------
* Remove WAIST_Link to use only WAIST (Fix "Either Interactive Marker or Natto-view appears, not together." `#97 <https://github.com/tork-a/rtmros_nextage/issues/97>`_).
* DIO Accessor:

  * Ignore tests for hand lighting when on simulation (Fix `#94 <https://github.com/tork-a/rtmros_nextage/issues/94>`_)
  * (DIO files) Minor improvement to api doc.
* Contributors: Isaac IY Saito

0.2.18 (2014-08-01)
-------------------
* (moveit_config) Default speed now moderately slow.
* Contributors: Isaac IY Saito

0.2.17 (2014-07-24)
-------------------
* (nextage_client) Adjusted to DIO spec change. Improve DIO methods. Add unit tests for DIO features.
* Contributors: Isaac IY Saito

0.2.16 (2014-07-24)
-------------------
* Specify min version of a dependency that are used in launch files.
* Add natto-view to simulation.
* Remove nextage_ros_bridge.launch that was only internal to other launch files. Delegate functionality to hironx_ros_bridge.launch
* (moveit_rviz) Correct fixed frame. This re-enables Interactive Marker to appear.
* (nextage_moveit_config) Add run_depend on hironx_moveit config (needed after https://github.com/tork-a/rtmros_nextage/commit/aa1c453c4ade5b9f44f94984f270a73e8e8e9376).
* Contributors: Isaac IY Saito

0.2.15 (2014-07-13)
-------------------
* (nextage_ros_bridge_real.launch) Init commit. This must be run when working with a real robot, instead of nextage_ros_bridge.launch. Fix `#79 <https://github.com/tork-a/rtmros_nextage/issues/79>`_
* Disable ServoController. NXO by default does not ship with servo-controlled hand.
* Enable natto-view.
* Contributors: Isaac IY Saito

0.2.14 (2014-06-20)
-------------------
* (nextage_client.py) adjust initial position to that of HIRONX, evens it up.
* Fix (`#73 <https://github.com/tork-a/rtmros_nextage/issues/73>`_)
* add more run_depends (`#71 <https://github.com/tork-a/rtmros_nextage/issues/71>`_)
* Contributors: Kei Okada, Isaac IY Saito

0.2.13 (2014-05-28)
-------------------
* Fix `#15 <https://github.com/tork-a/rtmros_nextage/issues/15>`_
* Contributors: Isaac IY Saito

0.2.12 (2014-05-06)
-------------------

0.2.11 (2014-03-05)
-------------------
* Fix `#53 <https://github.com/tork-a/rtmros_nextage/issues/53>`_
* Add the source text files of tutorials on ROS wiki. These are just a backup and not intended to be updated per every change made on ROS wiki. The location of the source of ROS wiki doc needs to be figured out (discussed in https://github.com/tork-a/rtmros_nextage/issues/12).
* Fix `#23 <https://github.com/tork-a/rtmros_nextage/issues/23>`_, `#46 <https://github.com/tork-a/rtmros_nextage/issues/46>`_
* Contributors: Isaac Isao Saito

0.2.10 (2014-02-18)
-------------------
* Use generic name for the robot instance. This enables users on the script commandline (eg. ipython) to run the same commands without asking them to specifically tell what robot they're using (eg. hiro, nxc). This is backward compatible so that users can still keep using `nxc`. See http://code.google.com/p/rtm-ros-robotics/source/detail?r=6926 for hironx.
* Install unittests for the first time.
* Contributors: Isaac Isao Saito

0.2.9 (2014-02-03)
------------------
* (nextage_ros_bridge) Fixed installation of missing py files
* Contributors: Isaac Isao Saito

0.2.8 (2014-02-03)
------------------
* Generalize hands DIO variables, and add a method to reassign them in the derived classes.
* Fix to issue `#9 <https://github.com/tork-a/rtmros_nextage/issues/9>`_ (https://github.com/tork-a/rtmros_nextage/issues/9)
* Adjust to the DIO assignment change.
* (test_hironx_derivedmethods_rostest.py) Tentative fix to enable to connect to real robot. Needs improvement later to port out embedded robot's info.
* Fixed handlight not function (wrong comparison of bool and str)
* Add more unittesting. Separate tests for hand since the type of testing for hands I'll write this time will be not necessarily general enough.
* Add tentative test file that checks cartesian
* (nextage_ros_bridge) Refactoring to separate modules per hand type, to allow more flexible hand tool combination. Not tested yet on a real robot and on simulation it isn't possible to test as of the moment.
* Contributors: Isao Isaac Saito

0.2.7 (2014-01-19)
------------------

0.2.6 (2014-01-13)
------------------
* (nextage_ros_bridge) Add missiong import
* Contributors: Isao Isaac Saito

0.2.5 (2013-12-25)
------------------
* Adjust to the change on hironx
* Contributors: Isao Isaac Saito

0.2.4 (2013-12-03)
------------------
* Bug fixes and refactoring.

0.2.3 (2013-11-05)
-----------

0.2.2 (2013-11-04)
-----------

0.2.1 (2013-10-31)
------------------
* Initial commit to the public repo (migrated from private repo)
