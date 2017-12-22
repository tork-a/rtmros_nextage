^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nextage_ros_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.3 (2017-12-22)
------------------
* add USE_HAND_JOINT_STATE_PUBLISHER argment, normal nextage does not need this (`#355 <https://github.com/tork-a/rtmros_nextage/issues/355>`_)
* add model cafe_tabel and mod ar_headcamera.launch to spawn gazebo models in the case of sim:=true (`#354 <https://github.com/tork-a/rtmros_nextage/issues/354>`_)
* Contributors: Kei Okada, Yosuke Yamamoto

0.8.1 (2017-09-28)
------------------
* Add files for manuals and tutorials / Changes for Gazebo head camera emulation (`#350 <https://github.com/tork-a/rtmros_nextage/issues/350>`_)
  * add nextage_moveit_sample.py of ryu yamamoto
  * mod name CAMERA_HEAD_R/L, remove CATKIN_IGNORE (forgot to add/rm them)
  * add AR codes, mod name CAMERA_HEAD_L/R, delete CATKIN_IGNORE
  * Merge branch 'impr/gazebo_ueye_fixtest' of git://github.com/130s/rtmros_nextage into gazebo_ueye_fixtest
  * [TEMP] Disable build on some pkgs for debugging purpose only. This MUST be reverted.
  * [ros_bridge][rviz.launch] Remove elements redundant to its parent launch file.
* Contributors: Isaac I.Y. Saito, Kei Okada, Yosuke Yamamoto

0.8.0 (2017-09-07)
------------------
* Fix for kinetic (`#347 <https://github.com/tork-a/rtmros_nextage/issues/347>`_)
  * fix .travis.yaml to run both indigo/kinetic with docker
  * fix install(CODE for kintic
* [ros_bridge] Missing dependency for using stereo vision. (`#331 <https://github.com/tork-a/rtmros_nextage/issues/331>`_ )
  * [ros_bridge] Missing dependency for using stereo vision.
    This dependency won't be necessary unless stereo vision is in use, but the advantage of having necessary pkg installed without the need for worrying surpasses disadvantage.
* [ros_bridge] Fix a launch file name to appropriately indicate "older than" 315.2.7. (`#328 <https://github.com/tork-a/rtmros_nextage/issues/328>`_)
    Hyphen here is used to indicate the file is intended for the mentioned version or larger/smaller.
  "*version-.launch" sounds like the launch is targeted for the version or larger while in fact we wanted to mean the opposite.
* add comments to functions in playPattern example codes (`#323 <https://github.com/tork-a/rtmros_nextage/issues/323>`_)
* Contributors: Isaac I.Y. Saito, Kei Okada, Yosuke Yamamoto

0.7.16 (2017-05-04)
-------------------
* [fix] Temporary revert getRTCList calling from the parent class, HIRONX `#317 <https://github.com/tork-a/rtmros_nextage/pull/317>`_
* [enhance][ros_bridge] Improve error message when no running ROS found. `#309 <https://github.com/tork-a/rtmros_nextage/pull/309>`_
* Contributors: Isaac I.Y. Saito

0.7.15 (2017-03-11)
-------------------

0.7.14 (2017-02-21)
-------------------
* [capability] hand camera now available on hrpsys simulator.
* Contributors: Kei Okada, Isaac I.Y. Saito

0.7.13 (2017-01-24)
-------------------

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
* Contributors: Isaac I.Y. Saito

0.7.9 (2016-10-13)
------------------
* [fix][nxo client] `#262 <https://github.com/tork-a/rtmros_nextage/issues/262>`_ by catching error when ros master is not running
* [fix] nextage_clinet with gazebo under no rtm environment.
* [capability] Activate ImpedanceController RTC. `#261 <https://github.com/tork-a/rtmros_nextage/issues/261>`_
* [improve] better initial hrpsys view
* Contributors: Kei Okada, Isaac I.Y. Saito

0.7.8 (2016-07-01)
------------------

0.7.7 (2016-05-16)
------------------
* [fix] Wrong collada file location for real.launch.
* [fix] Use NXO file instead of Hironx as a collision detector input.
* Contributors: Isaac I.Y. Saito

0.7.6 (2016-02-09)
------------------
* [improve] better view for hrpsys-simulator
* Contributors: Kei Okada

0.7.5 (2016-01-27)
------------------

0.7.4 (2016-01-26)
------------------

0.7.3 (2015-12-31)
------------------
* [sys] clean up nxo.test
* Contributors: Kei Okada, Isaac I.Y. Saito

0.7.2 (2015-11-02)
------------------

0.7.1 (2015-10-26)
------------------

0.6.6 (2015-10-17)
------------------

0.6.5 (2015-10-16)
------------------
* [Fix] stop air blows out when tools get ejected (fix `#158 <https://github.com/tork-a/rtmros_nextage/issues/158>`_)
* [Feat] Print reason why the command call errors.
* [Fix] fix ar.launch
* Contributors: Isaac IY Saito, Yutaka Kondo

0.6.4 (2015-10-02)
------------------
* [feat.] Add launch file for AR alvar 
* Contributors: Ryu Yamamoto

0.6.3 (2015-08-15)
------------------

0.6.2 (2015-05-12)
------------------
* (Feature) Add hands_ueye.launch for bringing up hand's ueye camera nodes.
* (Fix) [test_handlight.py] fix to pass the test, handlight (writeDigitalOutput always returns True in simulation https://github.com/fkanehiro/hrpsys-base/blob/master/python/hrpsys_config.py#L1284)
* Contributors: Kei Okada, Ryosuke Tajima

0.6.1 (2015-03-09)
------------------
* Start ROS clinent when the script begins
* Contributors: Kei Okada

0.6.0 (2015-02-03)
------------------
* [nextage_ros_bridge] Fix path for catkin build
* VRML stored location inside qnx is now NEXTAGE specific.
* (launch) Accept more as an argument. Remove a redundant collada file.
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

0.2.17 (2014-07-24)
-------------------
* (nextage_client) Adjusted to DIO spec change. Improve DIO methods. Add unit tests for DIO features.
* Contributors: Isaac IY Saito

0.2.16 (2014-07-24)
-------------------
* Specify min version of a dependency that are used in launch files.
* Add natto-view to simulation.
* Remove nextage_ros_bridge.launch that was only internal to other launch files. Delegate functionality to hironx_ros_bridge.launch
* Contributors: Isaac IY Saito

0.2.15 (2014-07-13)
-------------------
* (nextage_ros_bridge_real.launch) Init commit. This must be run when working with a real robot, instead of nextage_ros_bridge.launch. Fix `#79 <https://github.com/tork-a/rtmros_nextage/issues/79>`_
* Disable ServoController. NXO by default does not ship with servo-controlled hand.
* Contributors: Isaac IY Saito

0.2.14 (2014-06-20)
-------------------
* (nextage_client.py) adjust initial position to that of HIRONX, evens it up.
* Contributors: Isaac IY Saito

0.2.13 (2014-05-28)
-------------------

0.2.12 (2014-05-06)
-------------------
* Resolves `#48 <https://github.com/tork-a/rtmros_nextage/issues/48>`_
* Contributors: Isaac IY Saito

0.2.11 (2014-03-05)
-------------------
* fix to https://github.com/tork-a/rtmros_nextage/issues/53
* Add the source text files of tutorials on ROS wiki. These are just a backup and not intended to be updated per every change made on ROS wiki. The location of the source of ROS wiki doc needs to be figured out (discussed in https://github.com/tork-a/rtmros_nextage/issues/12).
* Contributors: Isaac Isao Saito

0.2.10 (2014-02-18)
-------------------
* Use generic name for the robot instance. This enables users on the script commandline (eg. ipython) to run the same commands without asking them to specifically tell what robot they're using (eg. hiro, nxc). This is backward compatible so that users can still keep using `nxc`. See http://code.google.com/p/rtm-ros-robotics/source/detail?r=6926 for hironx.
* Install unittests for the first time.
* Contributors: Isaac Isao Saito

0.2.9 (2014-02-03)
------------------
* Fixed installation of missing py files
* Contributors: Isao Isaac Saito

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
* (nextage_client.py) Adjust initial poses to what the manufacturing company defines as the standard.
* (nextage_client.py) Override pose (to more safer one) and method (to allow this class to choose which RT component to load).
* Improve nextage.py import order and source (based on https://github.com/tork-a/rtmros_nextage/issues/25#issuecomment-32332068)
* (nextage.py) Fix to https://github.com/tork-a/rtmros_nextage/issues/24
* Contributors: Isaac Isao Saito, Hajime Saito, Kei Okada

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
* fix same reason that we have in https://code.google.com/p/rtm-ros-robotics/source/detail?r=6039, need to modify .xml/.conf file during install process
* Change name of a launch file to adjust common practice in rtmros world.
* rename script (due to discussion in issue `#7 <https://github.com/130s/rtmros_nextage/issues/7>`_)
* Contributors: Isaac Isao Saito, Kei Okada

0.2.3 (2013-11-05)
-----------

0.2.2 (2013-11-04)
-----------
* add depends to nextage_description and check if nextage_description_SOURCE_DIR exsits
* nextage_ros_bridge) add appropriate comment for exception at hand
* nextage_ros_bridge) add nextage_ros_bridge_viz.launch that runs all the things needed for running robots (ros_bridge, RViz).

0.2.1 (2013-10-31)
------------------
* Initial commit to the public repo (migrated from private repo)
