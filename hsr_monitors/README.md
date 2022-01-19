# How to monitor HSR with RTAMT

## Setting up

ROS HSR pakages are necessary.

launch HSR simulator

```bash
roslaunch hsrb_gazebo_launch hsrb_megaweb2015_world.launch
```

With different terminal, you may start our catkin work space which git clone rtamt4ros.

```bash
cd <YOUR_CATKIN_WORK_SPACE>
catkin_make
source devel/setup.sh
```

## Safe case

```bash
roslaunch hsr_monitors hsr_monitor.launch rqt_plot:=true
```

Then order 2D navigation in rviz of HSR.

## Planner fail with fault injection

```bash
roslaunch hsr_monitors hsr_monitor.launch rqt_plot:=true
roslaunch hsr_monitors wrong_global_path.launch
```
