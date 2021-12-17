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
4. Install python packages
```
conda env create --file=environment.yml
```
(Or if this breaks, create and activate cosim environment and install the packages found in environment.yml or as necessary based on error messages)
```
conda activate cosim
```

## Run simulation
1. rosrun cosim demo\_node.py --platoon "av human\*5"
  *Notation for declaring the platoon is same as in https://github.com/nathanlct/trajectory-training-icra; vehicles can also be listed one by one, and the current options are "av" and "human"*
2. View .png graphs generated in cosim/scripts: {}\_vs\_time.png {xpos, ypos, vel, headway, rel\_vel, v\_acts, v\_acts\_cmd\_vels, v\_refs}
