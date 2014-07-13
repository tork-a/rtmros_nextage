^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtmros_nextage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
