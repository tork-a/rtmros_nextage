^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nextage_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.15 (2017-03-11)
-------------------
* [fix] Model file copyrights.

0.7.14 (2017-02-21)
-------------------
* [capability] hand camera now available on hrpsys simulator.
* Contributors: Kei Okada

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
* [fix] do not have to fix /WAIST to /world, if you want run static_transform_publisher, see `#235 <https://github.com/tork-a/rtmros_nextage/issues/235>`_
* [improve] set ARM_JOINT0 max effor to 150, ARM_JOINT1 max effor to 200, may be this is larget than real robot
* [improve] set mass of base link to super heavy
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
* [gazebo] Add non-zero mass so that links are not ignored
* NextageOpen.urdf: add transmission and ros_control
* set depend from nextage_description to nextage_gazebo is not a good idea, nextage_gazbeo should depends on nextage_description
* HEAD_JOINT1_Link is too small, use default settings http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model
* [gazebo] Add non-zero mass so that links are not ignored
* Add Gazebo package. So far model not shown, and model seems to keep falling.
* Contributors: Isaac I.Y. Saito, Kei Okada

0.7.3 (2015-12-31)
------------------

0.7.2 (2015-11-02)
------------------

0.7.1 (2015-10-26)
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
* Correct changelogs to apply the important announcement.
* Contributors: Isaac IY Saito

0.6.1 (2015-03-09)
------------------

0.6.0 (2015-02-03)
------------------
* (IMPORTANT) VRML stored location inside qnx has now become NEXTAGE specific. Please see https://github.com/tork-a/rtmros_nextage/issues/153 for possible required actions.
* (launch) Accept more as an argument. Remove a redundant collada file.
* Contributors: Isaac IY Saito

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
* Remove WAIST_Link to use only WAIST (Fix `#97 <https://github.com/tork-a/rtmros_nextage/issues/97>`_).
* Contributors: Isaac IY Saito

0.2.18 (2014-08-01)
-------------------

0.2.17 (2014-07-24)
-------------------

0.2.16 (2014-07-24)
-------------------

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

0.2.9 (2014-02-03)
------------------

0.2.8 (2014-02-03)
------------------

0.2.7 (2014-01-19)
------------------

0.2.6 (2014-01-13)
------------------

0.2.5 (2013-12-25)
------------------

0.2.4 (2013-12-03)
------------------

0.2.3 (2013-11-05)
-----------

0.2.2 (2013-11-04)
-----------
* install models, urdf, www
* Adjust to web-tablet version discussed at https://github.com/start-jsk/open-industrial-controllers/issues/121. Please revert this if this causes any bad effect.

0.2.1 (2013-10-31)
------------------
* Initial commit to the public repo (migrated from private repo)
