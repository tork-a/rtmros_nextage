nextage_gazebo
==============

How to setup
------------

1. install from source
```
$ source /opt/ros/noetic/setup.bash                   ;; Setup ROS environment
$ mkdir -p ~/ws_nextage/src                           ;; Create workspace
$ cd ~/ws_nextage/src/                                ;; Move to workspace
$ git clone https://github.com/tork-a/rtmros_nextage  ;; Checkout source code
$ rosdep install -r --from-paths src --ignore-src     ;; Install dependency packages
;;
;; This command outputs following errors, but you can ignore
;;
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
nextage_moveit_config: Cannot locate rosdep definition for [hironx_moveit_config]
nextage_ros_bridge: Cannot locate rosdep definition for [ar_track_alvar]
nextage_calibration: Cannot locate rosdep definition for [turtlebot_description]
Continuing to install resolvable dependencies...
;;
```

2. Build `nextage_gazebo` package

Note that normal `catkin build` will outputs build erros, so run following commands
```
$ catkin build nextage_gazebo nextage_description --no-deps
```

How to use
----------

1. Setup workspace environment
```
$ source ~/ws_nextage/devel/setup.bash
```

2. Run gazebo simulator
```
$ roslaunch nextage_gazebo nextage_world.launch 
```

3. How to move arms

You can use `rqt-joint-trajectory-controller` to examine controllers
```
$ sudo apt install ros-noetic-rqt-joint-trajectory-controller
$ source /opt/ros/noetic/setup.bash 
$ rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

4. Check list of topics
```
$ rostopic list
/CAMERA_HEAD_L/camera_info
/CAMERA_HEAD_L/image_raw
/CAMERA_HEAD_L/image_raw/compressed
/CAMERA_HEAD_L/image_raw/compressed/parameter_descriptions
/CAMERA_HEAD_L/image_raw/compressed/parameter_updates
/CAMERA_HEAD_L/image_raw/compressedDepth
/CAMERA_HEAD_L/image_raw/compressedDepth/parameter_descriptions
/CAMERA_HEAD_L/image_raw/compressedDepth/parameter_updates
/CAMERA_HEAD_L/image_raw/theora
/CAMERA_HEAD_L/image_raw/theora/parameter_descriptions
/CAMERA_HEAD_L/image_raw/theora/parameter_updates
/CAMERA_HEAD_L/parameter_descriptions
/CAMERA_HEAD_L/parameter_updates
/CAMERA_HEAD_R/camera_info
/CAMERA_HEAD_R/image_raw
/CAMERA_HEAD_R/image_raw/compressed
/CAMERA_HEAD_R/image_raw/compressed/parameter_descriptions
/CAMERA_HEAD_R/image_raw/compressed/parameter_updates
/CAMERA_HEAD_R/image_raw/compressedDepth
/CAMERA_HEAD_R/image_raw/compressedDepth/parameter_descriptions
/CAMERA_HEAD_R/image_raw/compressedDepth/parameter_updates
/CAMERA_HEAD_R/image_raw/theora
/CAMERA_HEAD_R/image_raw/theora/parameter_descriptions
/CAMERA_HEAD_R/image_raw/theora/parameter_updates
/CAMERA_HEAD_R/parameter_descriptions
/CAMERA_HEAD_R/parameter_updates
/clock
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/performance_metrics
/gazebo/set_link_state
/gazebo/set_model_state
/head_controller/command
/head_controller/follow_joint_trajectory_action/cancel
/head_controller/follow_joint_trajectory_action/feedback
/head_controller/follow_joint_trajectory_action/goal
/head_controller/follow_joint_trajectory_action/result
/head_controller/follow_joint_trajectory_action/status
/head_controller/gains/HEAD_JOINT0/parameter_descriptions
/head_controller/gains/HEAD_JOINT0/parameter_updates
/head_controller/gains/HEAD_JOINT1/parameter_descriptions
/head_controller/gains/HEAD_JOINT1/parameter_updates
/head_controller/state
/joint_states
/larm_controller/command
/larm_controller/follow_joint_trajectory_action/cancel
/larm_controller/follow_joint_trajectory_action/feedback
/larm_controller/follow_joint_trajectory_action/goal
/larm_controller/follow_joint_trajectory_action/result
/larm_controller/follow_joint_trajectory_action/status
/larm_controller/gains/LARM_JOINT0/parameter_descriptions
/larm_controller/gains/LARM_JOINT0/parameter_updates
/larm_controller/gains/LARM_JOINT1/parameter_descriptions
/larm_controller/gains/LARM_JOINT1/parameter_updates
/larm_controller/gains/LARM_JOINT2/parameter_descriptions
/larm_controller/gains/LARM_JOINT2/parameter_updates
/larm_controller/gains/LARM_JOINT3/parameter_descriptions
/larm_controller/gains/LARM_JOINT3/parameter_updates
/larm_controller/gains/LARM_JOINT4/parameter_descriptions
/larm_controller/gains/LARM_JOINT4/parameter_updates
/larm_controller/gains/LARM_JOINT5/parameter_descriptions
/larm_controller/gains/LARM_JOINT5/parameter_updates
/larm_controller/state
/rarm_controller/command
/rarm_controller/follow_joint_trajectory_action/cancel
/rarm_controller/follow_joint_trajectory_action/feedback
/rarm_controller/follow_joint_trajectory_action/goal
/rarm_controller/follow_joint_trajectory_action/result
/rarm_controller/follow_joint_trajectory_action/status
/rarm_controller/gains/RARM_JOINT0/parameter_descriptions
/rarm_controller/gains/RARM_JOINT0/parameter_updates
/rarm_controller/gains/RARM_JOINT1/parameter_descriptions
/rarm_controller/gains/RARM_JOINT1/parameter_updates
/rarm_controller/gains/RARM_JOINT2/parameter_descriptions
/rarm_controller/gains/RARM_JOINT2/parameter_updates
/rarm_controller/gains/RARM_JOINT3/parameter_descriptions
/rarm_controller/gains/RARM_JOINT3/parameter_updates
/rarm_controller/gains/RARM_JOINT4/parameter_descriptions
/rarm_controller/gains/RARM_JOINT4/parameter_updates
/rarm_controller/gains/RARM_JOINT5/parameter_descriptions
/rarm_controller/gains/RARM_JOINT5/parameter_updates
/rarm_controller/state
/rosout
/rosout_agg
/tf
/tf_static
/torso_controller/command
/torso_controller/follow_joint_trajectory_action/cancel
/torso_controller/follow_joint_trajectory_action/feedback
/torso_controller/follow_joint_trajectory_action/goal
/torso_controller/follow_joint_trajectory_action/result
/torso_controller/follow_joint_trajectory_action/status
/torso_controller/gains/CHEST_JOINT0/parameter_descriptions
/torso_controller/gains/CHEST_JOINT0/parameter_updates
/torso_controller/state

```