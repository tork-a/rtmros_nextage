^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nextage_calibration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.0 (2017-09-07)
------------------
* Fix for kinetic (`#347 <https://github.com/tork-a/rtmros_nextage/issues/347>`_)
  * fix .travis.yaml to run both indigo/kinetic with docker
  * add time.sleep before call self.tflistener.getFrameString() to wait to initialize listener, see https://github.com/ros/geometry/issues/152
  * increase time-limit
* Contributors: Kei Okada

0.7.16 (2017-05-04)
-------------------
* [fix] error "libgazebo_ros_openni_kinect.so: cannot open shared object file" `#324 <https://github.com/tork-a/rtmros_nextage/pull/324>`_
* [maintenance] Add simple testcases.
* Contributors: Kei Okada, Yamamoto Yosuke, Isaac I.Y. Saito

0.7.15 (2017-03-11)
-------------------
* [capability] add package nextage_calibration
* Contributors: Yamamoto Yosuke

0.7.14 (2017-02-21)
-------------------

