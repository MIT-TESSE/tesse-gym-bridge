# tesse-gym-bridge

An interface between [tesse-gym](https://github.mit.edu/TESS/tesse-gym) and ROS, allowing an RL agent to operate in TESSE while using results from perception modules.

<div align="center">
  <img src="docs/tesse_kimera_gym_2.gif">
</div>

## Installation

1. Clone and build this repo

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
wstool merge gym-ros-interface/install/tesse_gym_bridge.rosinstall 
cd ..

# compile
catkin build

# source workspace
source ~/catkin_ws/devel/setup.bash
```

#### To run an RL Agent with Kimera-VIO and Kimera-Semantics, the following steps are required

2. Install [Kimera-VIO-ROS](https://github.com/MIT-SPARK/Kimera-VIO-ROS) and [Kimera-Semantics](https://github.com/MIT-SPARK/Kimera-Semantics)

Please refer to their installation guide.


3. Install [tesse-interface](https://github.mit.edu/TESS/tesse-interface/tree/feature/rgb-metadata) and switch to branch `feature/rgb-metadata`

Follow the tesse-interface [installation instructions](https://github.mit.edu/TESS/tesse-interface) then:

```sh
cd ~/catkin_ws/src/tesse-interface
git checkout feature/rgb-metadata 
```

4. Install [tesse-gym](https://github.mit.edu/TESS/tesse-gym)
This isn't a ROS package, so you don't have to install it into your `catkin_ws`

```sh
cd ~/your_directory
git clone git@github.mit.edu:TESS/tesse-gym.git
cd tesse-gym
python setup.py develop
```


## Usage

To run TESSE, Kimera-VIO, and Kimera-Semantics, and the required interfaces

```sh
roslaunch tesse_gym_bridge run_kimera_tesse.launch tesse_path:=TESSE_BUILD_PATH
```

To control the TESSE agent through the Gym environment, run see the [move-agent](./notebooks/move-agent.ipynb) notebook

## Disclaimer

DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.

This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Under Secretary of Defense for Research and Engineering.

© 2020 Massachusetts Institute of Technology.

MIT Proprietary, Subject to FAR52.227-11 Patent Rights - Ownership by the contractor (May 2014)

The software/firmware is provided to you on an As-Is basis

Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
