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

## System all green case

```bash
roslaunch hsr_monitors hsr_monitor.launch rqt_plot:=true
```

Then order 2D navigation in rviz of HSR.

## Planner fail with fault injection

```bash
roslaunch hsr_monitors hsr_monitor.launch rqt_plot:=true
roslaunch hsr_monitors wrong_global_path.launch
```

## Offline analysis with rosbag

### Recoding

Please refer [System all green case](#system-all-green-case) first.

Then we may recode it.

- all data

    ```bash
    rosbag record -a -O <YOUR_ROSBAG_NAME.bag>
    ```

    example

    ```bash
    rosbag record -a -O $(rospack find hsr_monitors)/rosbag/system_all_green.bag
    ```

- only under /rtamt data

    ```bash
    rosbag record -e "/rtamt/(.*)" -O <YOUR_ROSBAG_NAME.bag>
    ```

    example

    ```bash
    rosbag record -e "/rtamt/(.*)" -O $(rospack find hsr_monitors)/rosbag/system_all_green.bag
    ```

### Viweing

```bash
rqt_bag <YOUR_ROSBAG_NAME.bag>
```

example

```bash
rqt_bag "/rtamt/(.*)" -O $(rospack find hsr_monitors)/rosbag/system_all_green.bag
```
