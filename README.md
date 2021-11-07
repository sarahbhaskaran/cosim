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
3. Start controller ros nodes
```
roscore
roslaunch transfer_pkg rl0719_readonly.launch readonly:=false
```
This is all for main branch for now; when using acceleration model, start running that rosnode as well.

## Run simulation
1. `rosrun cosim demo_node.py`

## View results
1. 
```
cd scripts
python graph.py
```
