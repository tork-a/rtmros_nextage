^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nextage_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.3 (2017-12-22)
------------------
* add model cafe_tabel and mod ar_headcamera.launch to spawn gazebo models in the case of sim:=true (`#354 <https://github.com/tork-a/rtmros_nextage/issues/354>`_)
* Contributors: Yosuke Yamamoto

0.8.1 (2017-09-28)
------------------
* Add files for manuals and tutorials / Changes for Gazebo head camera emulation (`#350 <https://github.com/tork-a/rtmros_nextage/issues/350>`_)
  * increase retry time
  * gz.test: set verbose to launch nextage_world.launch
  * time-limit 10 is too small for hz test
  * move ar marker model files to nextage_gazebo/models from nextage_moveit_config/models
  * mod name CAMERA_HEAD_R/L, remove CATKIN_IGNORE (forgot to add/rm them)
  * [Gazebo] Missing dependency for using camera plugin.
  * [Gazebo] Unwind the frequency test.
    hz requirement for Gazebo sensor plugin on Travis CI needs to be generous. There's a known issue about Gazebo's sensor frequency https://bitbucket.org/osrf/gazebo/pull-requests/2502/make-sure-cameras-fps-is-strictly-applied/diff and with the lack of GPU or higher CPU, the sensor output is significantly slow on Travis CI.
  * [gazebo] Add headmount cameras.
  * [gazebo] Remove a redundant model file that was added by mistake in https://github.com/tork-a/rtmros_nextage/pull/223.
  * [gazebo] Cleanup CMakeLists.txt, package.xml.
* Contributors: Isaac Saito, Kei Okada, Yosuke Yamamoto

0.8.0 (2017-09-07)
------------------

0.7.16 (2017-05-04)
-------------------
* [enhance] nextage_gazebo/launch/nextage_gazebo_control.launch : add --shutdown-timeout
* [enhance] nextage_gazebo/test/gz.test: add retry=2
* Contributors: Isaac I.Y. Saito, Kei Okada

0.7.15 (2017-03-11)
-------------------

0.7.14 (2017-02-21)
-------------------

0.7.13 (2017-01-24)
-------------------

0.7.12 (2017-01-10)
-------------------

0.7.11 (2016-11-05)
-------------------

0.7.10 (2016-10-19)
-------------------

0.7.9 (2016-10-13)
------------------
* [fix][test] gz.test must be one file to avoid collsion on gazebo master uri
* [fix] Missing installation
* [fix] : test/test_nxo_gz.py, because of changeing contorller parameter?
* [improve] tune pid parameter for gazebo
* Contributors: Kei Okada

0.7.8 (2016-07-01)
------------------

0.7.7 (2016-05-16)
------------------

0.7.6 (2016-02-09)
------------------

0.7.5 (2016-01-27)
------------------
* [fix] fix namespace so that existing app can operate the robot on Gazebo
* Contributors: Kei Okada

0.7.4 (2016-01-26)
------------------
* need to wait for finishing go_initial.py (https://github.com/tork-a/rtmros_nextage/pull/223/files#diff-16b25951a50b1e80569929d32a09102bR14)
* pass arg GUI
* [gz] Retry tests upon failure
* [gz] Add small unit tests
* [gz] Minor formatting
* [gazebo] Add non-zero mass so that links are not ignored
* update control parameters
* http://answers.ros.org/question/200777/using-both-jointtrajectorycontroller-and-jointpositioncontroller-at-same-time-in-ros_control, we may not need both position controller and trajectory controller
* use NextageOpen name space, use robot name. Not sure if this is necessary
* package.xml: add depends to ros_controllers
* set depend from nextage_description to nextage_gazebo is not a good idea, nextage_gazbeo should depends on nextage_description
* [gazebo] Add non-zero mass so that links are not ignored
* Add Gazebo package. So far model not shown, and model seems to keep falling.
* Contributors: Isaac I.Y. Saito, Kei Okada

0.7.3 (2015-12-31)
------------------

0.7.2 (2015-11-02)
------------------

0.7.1 (2015-10-26)
------------------

0.7.0 (2015-10-21)
------------------

0.6.6 (2015-10-17)
------------------

0.6.5 (2015-10-16)
------------------

0.6.4 (2015-10-02)
------------------

0.6.3 (2015-08-15)
------------------

0.6.2 (2015-05-12)
------------------

0.6.1 (2015-03-09)
------------------

0.6.0 (2015-02-03)
------------------

0.5.3 (2014-11-13)
------------------

0.5.2 (2014-11-03)
------------------

0.5.1 (2014-10-17)
------------------

0.4.2 (2014-10-01)
------------------

0.4.1 (2014-09-03)
------------------

0.2.18 (2014-08-01)
-------------------

0.2.17 (2014-07-24 14:01)
-------------------------

0.2.16 (2014-07-24 10:09)
-------------------------

0.2.15 (2014-07-13)
-------------------

0.2.14 (2014-06-20)
-------------------

0.2.13 (2014-05-28)
-------------------

0.2.12 (2014-05-06)
-------------------

0.2.11 (2014-03-05)
-------------------

0.2.10 (2014-02-18)
-------------------

0.2.9 (2014-02-03 12:34)
------------------------

0.2.8 (2014-02-03 03:12)
------------------------

0.2.7 (2014-01-19)
------------------

0.2.6 (2014-01-13)
------------------

0.2.5 (2013-12-25)
------------------

0.2.4 (2013-12-03)
------------------

0.2.3 (2013-11-05)
------------------

0.2.2 (2013-11-04)
------------------

0.2.1 (2013-10-31)
------------------

0.2.0 (2013-10-30)
------------------
