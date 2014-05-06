^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nextage_ros_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
