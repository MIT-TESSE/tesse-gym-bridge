# tesse-gym-bridge

An interface between [tesse-gym](https://github.com/MIT-TESSE/tesse-gym) and ROS, allowing an RL agent to operate in TESSE while using results from perception modules.

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
git clone https://github.com/MIT-TESSE/tesse-gym-bridge.git

# install dependencies
wstool init
wstool merge tesse-gym-bridge/install/tesse_gym_bridge.rosinstall 
cd ..

# compile
catkin build

# source workspace
source ~/catkin_ws/devel/setup.bash
```

#### To run an RL Agent with Kimera-VIO and Kimera-Semantics, the following steps are required

2. Install [Kimera-VIO-ROS](https://github.com/MIT-SPARK/Kimera-VIO-ROS) and [Kimera-Semantics](https://github.com/MIT-SPARK/Kimera-Semantics)

Please refer to their installation guide.

3. Install [tesse-interface](https://github.com/MIT-TESSE/tesse-interface) 

4. Install [tesse-ros-bridge](https://github.com/MIT-TESSE/tesse-ros-bridge)

5. Install [tesse-segmentation-ros](https://github.com/MIT-TESSE/tesse-segmentation-ros)



## Usage

To run TESSE, Kimera-VIO, and Kimera-Semantics, and the required interfaces start the simulator and then run:

```sh
roslaunch tesse_gym_bridge run_goseek_perception.launch
```

To control the TESSE agent through the Gym environment, see the [move-agent](./notebooks/move-agent.ipynb) notebook. This requires installing [tesse-gym](https://github.com/MIT-TESSE/tesse-gym) in a Python 3.7 environment. 

## Disclaimer

DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.

This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Under Secretary of Defense for Research and Engineering.

© 2020 Massachusetts Institute of Technology.

MIT Proprietary, Subject to FAR52.227-11 Patent Rights - Ownership by the contractor (May 2014)

The software/firmware is provided to you on an As-Is basis

Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
