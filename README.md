# rtamt4ros

Real-time monitoring tool for ROS applications that uses the rtamt library

## Install back-ends

### ROS kinectic + Ubuntu16.04

Please see ROS kinectic page.
<http://wiki.ros.org/kinetic/Installation/Ubuntu>

### install rtamt

Please see rtamt tool repository.
<https://github.com/nickovic/rtamt>
rtamt is considered as working on ROS, while the tool is very independent and flexisble from ROS.

## Install rtamt for ROS

### make catkin_ws

You can set-up catkin_ws at any place.

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

### clone

add the rtamt for ROS repo on the src folder in catkin_ws.

```bash
cd ~/catkin_ws/src
git clone https://github.com/nickovic/rtamt4ros
```

### build

```bash
cd ~/catkin_ws
catkin_make
```

## Run

### basic example

```bash
cd ~/catkin_ws
source devel/setup.sh
roslaunch rtamt4ros ros_stl_monitor.launch
```

### API example

```bash
cd ~/catkin_ws
source devel/setup.sh
roslaunch rtamt4ros ros_stl_monitor_with_API.launch
```

### distributed example

```bash
cd ~/catkin_ws
source devel/setup.sh
roslaunch rtamt4ros decomposed_spec.launch
```

### rosbag example

```bash
cd ~/catkin_ws
source devel/setup.sh
roslaunch rtamt4ros ros_stl_monitor_with_rosbag.launch
```
