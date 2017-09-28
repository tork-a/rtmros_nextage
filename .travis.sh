$!/bin/bash

set -x

echo "Environment Variables"
echo "CI_SOURCE_PATH=$CI_SOURCE_PATH"
echo "REPOSITORY_NAME=$REPOSITORY_NAME"
echo "DEB_REPOSITORY=$DEB_REPOSITORY"
echo "CI_ROS_DISTRO=$CI_ROS_DISTRO"
echo "DISTRO=$DISTRO"
env | grep ROS

apt-get update -qq
apt-get install -qq -y sudo wget git lsb-release

function error {
## after_failure:
  catkin_test_results --verbose build
  exit 1
}

trap error ERR

## before_install:
  # use host xserver
sudo apt-get -y -qq install xvfb mesa-utils
export DISPLAY=:99
Xvfb  $DISPLAY -ac -screen 0 1024x768x24 &
sleep 3 # wait x server up
glxinfo
####### https://github.com/ros-planning/moveit/pull/581
if [ "$CI_ROS_DISTRO" == "kinetic" ]; then
    apt-get install -y python-pyassimp
    sed -i 's@load, load_mem, release, dll@load, release, dll@' /usr/lib/python2.7/dist-packages/pyassimp/core.py
    cat -n /usr/lib/python2.7/dist-packages/pyassimp/core.py
    ## fix "A new DepthBuffer for a RenderTarget was created, but after creation" problem
    ## http://answers.gazebosim.org/question/9395/gazebo-crashes-when-inserting-camera-in-virtual-machine-with-ubuntu-vivid-ros-jade/
    ## https://bitbucket.org/osrf/gazebo/issues/1837/vmware-rendering-z-ordering-appears-random#comment-31756295
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    sudo apt-get update -qq
    sudo apt-get install -qq -y gazebo7
fi
#######
## install:
sudo sh -c "echo \"deb ${DEB_REPOSITORY} `lsb_release -cs` main\" > /etc/apt/sources.list.d/ros-latest.list"
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
sudo apt-get install -qq -y python-rosdep python-catkin-tools
sudo rosdep init
rosdep update
  # Use rosdep to install all dependencies (including ROS itself)
rosdep install --from-paths ./ -i -y -q --rosdistro $CI_ROS_DISTRO
## script:
source /opt/ros/$CI_ROS_DISTRO/setup.bash
mkdir -p $CATKIN_WS_SRC
ln -s $TRAVIS_BUILD_DIR $CATKIN_WS_SRC
cd $CATKIN_WS
catkin init
  # Enable install space
catkin config --install
  # Build [and Install] packages
catkin build --limit-status-rate 0.1 --no-notify -DCMAKE_BUILD_TYPE=Release
  # Build tests
catkin build --limit-status-rate 0.1 --no-notify --make-args tests
  # Run tests
####### kinteic has problem on runnning gazebo with image on headless mode????
if [ "$CI_ROS_DISTRO" == "kinetic" ]; then
  touch src/rtmros_nextage/nextage_calibration/CATKIN_IGNORE
fi
#######
catkin run_tests -j1 -p1
  # check test (this only works from indigo onward)
catkin_test_results build
