This is the guide for pff_diff_robot package

# 1. ROS Environment Setup

### 1.1 Install ROS

[Official Guide]( http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

Quick Reference
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall
```

### 1.2 Install Teleop Package

Most of dependencies are already included. However, you still need to install the package for keyboard interface.
```bash
sudo apt-get install ros-kinetic-teleop-twist-keyboard
```

### 1.3 Build The Package

Now you can download this package and build it use catkin_make.
```bash
https://github.com/xuanmo0213/ROS-PFF.git
cd pff-workspace;
catkin_make;
source ./devel/setup.bash
```

### 1.4 Run The Node

When package is built, you can run node using robot.lauch. 
A RViz window will boot up, keep terminal active to ensure the keyboard input is recorded.
```bash
roslaunch pff_diff_robot robot.lauch [Optional]setmode:=$mode [Optional]setsize:=$size
$mode could be "keyboard", "circle" or "square".
$size could be a float for circle diameter and square size.
```
# 2. Node Topics And Parameters

## 2.1 Topics

| Topic | Msg | Description |
|:---:|:---:|:---:|
|`"/cmd_vel"`|`geometry_msgs.msg` | Comunicate for teleop |
| `"/odom"` | `nav_msgs/Odometry` | Publish transform from `/base_link` link to `/map` |
| `"/joint_states"` | `sensor_msgs/JointState` | Publish joint transform from joint_wheel_left and joint_wheel_right to map |
| `"/visualization_marker"` | `visualization_msgs.msg` | Publish to `/map` |

## 2.2 Parameters

| Parameter Name | Type | Default Value | Optional and Description |
|:---:|:---:|:---:|:---:|
| `rate` | double | `1.0` | Set rate for robot |
| `setmode` | string | `keyboard` | Select mode: 'keyboard' for keyboard control, 'circle' for run in circle, 'square' for run in square |
| `setsize` | double | `1.0` | Diameter of Circle and Length for Square Size |
| `rviz_config` | path | `$(find pff_diff_robot)/config/model.rviz` | Load config file for rviz |
| `model` | path | `$(find pff_diff_robot)/config/diff_robot.urdf"/>` | Load urdf file for robot |


# 3 Examples

### 3.1 Run node with default parameters. This will be keyboard mode.

```bash
roslaunch pff_diff_robot robot.launch
```

### 3.2 Run node in keyboard mode.

```bash
roslaunch pff_diff_robot robot.launch setmode:=keyboard
```

### 3.3 Run node in circle mode with size

```bash
roslaunch pff_diff_robot robot.launch setmode:=circle setsize:=2.0
```

### 3.4 Run node in square mode with size

```bash
roslaunch pff_diff_robot robot.launch setmode:=square setsize:=2.0
```


