# gym-ros-interface

An interface between [tesse-gym](https://github.mit.edu/TESS/tesse-gym) and ROS. 
This package takes requests from [tesse-interface](https://github.mit.edu/TESS/tesse-interface) (used by tesse-gym) and responds with data from rostopics, 
allowing for inputs from sources such as [Kimera](https://github.com/MIT-SPARK/Kimera-VIO-ROS).

## Installation

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

## Disclaimer

Distribution authorized to U.S. Government agencies and their contractors. Other requests for this document shall be referred to the MIT Lincoln Laboratory Technology Office.

This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Under Secretary of Defense for Research and Engineering.

© 2019 Massachusetts Institute of Technology.

The software/firmware is provided to you on an As-Is basis

Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
