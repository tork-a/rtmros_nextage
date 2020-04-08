$!/bin/bash

set -x

function travis_time_start {
    set +x
    TRAVIS_START_TIME=$(date +%s%N)
    TRAVIS_TIME_ID=$(cat /dev/urandom | tr -dc 'a-z0-9' | fold -w 8 | head -n 1)
    TRAVIS_FOLD_NAME=$1
    echo -e "${ANSI_CLEAR}traivs_fold:start:$TRAVIS_FOLD_NAME"
    echo -e "${ANSI_CLEAR}traivs_time:start:$TRAVIS_TIME_ID${ANSI_BLUE}>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>${ANSI_RESET}"
    set -x
}
function travis_time_end {
    set +x
    _COLOR=${1:-32}
    TRAVIS_END_TIME=$(date +%s%N)
    TIME_ELAPSED_SECONDS=$(( ($TRAVIS_END_TIME - $TRAVIS_START_TIME)/1000000000 ))
    echo -e "traivs_time:end:$TRAVIS_TIME_ID:start=$TRAVIS_START_TIME,finish=$TRAVIS_END_TIME,duration=$(($TRAVIS_END_TIME - $TRAVIS_START_TIME))\n${ANSI_CLEAR}"
    echo -e "traivs_fold:end:$TRAVIS_FOLD_NAME\e[${_COLOR}m<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<${ANSI_RESET}"
    echo -e "${ANSI_CLEAR}\e[${_COLOR}mFunction $TRAVIS_FOLD_NAME takes $(( $TIME_ELAPSED_SECONDS / 60 )) min $(( $TIME_ELAPSED_SECONDS % 60 )) sec${ANSI_RESET}"
    set -x
}

echo "Environment Variables"
echo "CI_SOURCE_PATH=$CI_SOURCE_PATH"
echo "REPOSITORY_NAME=$REPOSITORY_NAME"
echo "DEB_REPOSITORY=$DEB_REPOSITORY"
echo "CI_ROS_DISTRO=$CI_ROS_DISTRO"
echo "DISTRO=$DISTRO"
env | grep ROS

travis_time_start setup

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

travis_time_end
travis_time_start setup_pyassimp_gazebo

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

travis_time_end
travis_time_start rosdep

#######
## install:
sudo sh -c "echo \"deb ${DEB_REPOSITORY} `lsb_release -cs` main\" > /etc/apt/sources.list.d/ros-latest.list"
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
sudo apt-get install -qq -y python-rosdep python-catkin-tools
sudo rosdep init
rosdep update --include-eol-distros
  # Use rosdep to install all dependencies (including ROS itself)
rosdep install --from-paths ./ -i -y -q --rosdistro $CI_ROS_DISTRO

travis_time_end
travis_time_start catkin_build

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

travis_time_end
travis_time_start catkin_build_tests

catkin build --limit-status-rate 0.1 --no-notify --make-args tests

travis_time_end
travis_time_start catkin_run_tests

  # Run tests
####### kinteic has problem on runnning gazebo with image on headless mode????
if [ "$CI_ROS_DISTRO" == "kinetic" ]; then
  touch src/rtmros_nextage/nextage_calibration/CATKIN_IGNORE
fi
#######
catkin run_tests -j1 -p1
  # check test (this only works from indigo onward)
catkin_test_results build

travis_time_end
