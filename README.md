# gym-ros-interface

An interface between [tesse-gym](https://github.mit.edu/TESS/tesse-gym) and ROS, allowing an RL agent to operate in TESSE while using results from thrid-party perception modules.

<div align="center">
  <img src="docs/tesse_kimera_gym_2.gif">
</div>

## Installation
1. Clone and build the repo

```sh
# setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init

# clone repo
cd src
git clone git@github.mit.edu:TESS/gym-ros-interface.git

# install dependencies
wstool init
wstool merge gym-ros-interface/install/gym_ros_interface.rosinstall 
cd ..

# compile
catkin build

# source workspace
source ~/catkin_ws/devel/setup.bash
```

## Usage

To run TESSE, Kimera-VIO, and Kimera-Semantics, and the required interfaces 

```sh
roslaunch gym_ros_interface run_kimera_tesse.launch
```

To control the TESSE agent through the Gym environment, run see the [move-agent](https://github.mit.edu/TESS/gym-ros-interface/blob/feature/unified-launch/notebooks/move-agent.ipynb) notebook

## Disclaimer

Distribution authorized to U.S. Government agencies and their contractors. Other requests for this document shall be referred to the MIT Lincoln Laboratory Technology Office.

This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Under Secretary of Defense for Research and Engineering.

© 2019 Massachusetts Institute of Technology.

The software/firmware is provided to you on an As-Is basis

Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
