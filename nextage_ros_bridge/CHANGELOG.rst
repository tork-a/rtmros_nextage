^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nextage_ros_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
