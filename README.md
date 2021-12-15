# cosim

## Setup

1. Put repositories in catkin workspace, including this one
```
cd ~/catkin_ws/src
git clone git@github.com:sarahbhaskaran/cosim.git
git clone git@github.com:jmscslgroup/hoffmansubsystem
git clone git@github.com:jmscslgroup/followerstoppermax4rl
git clone git@github.com:jmscslgroup/followerstoppermax
git clone git@github.com:jmscslgroup/followerstopperth
git clone git@github.com:jmscslgroup/micromodel
git clone git@github.com:jmscslgroup/trajectory_07_05_2021_real
git clone git@github.com:jmscslgroup/velocity_controller
git clone git@github.com:jmscslgroup/integrator
git clone git@github.com:jmscslgroup/margin
git clone git@github.com:jmscslgroup/can_to_ros
git clone git@github.com:jmscslgroup/transfer_pkg
```
2. Build packages
```
cd ~/catkin_ws
catkin_make
source devel/setup.sh
```
3. Start ROS
```
roscore
```

## Run simulation
1. rosrun cosim demo_node.py --platoon "av human\*5"
  *Notation for declaring the platoon is same as in https://github.com/nathanlct/trajectory-training-icra; vehicles can also be listed one by one, and the current options are "av" and "human"*
3. Graph .npy files generated in cosim/scripts/data
