# beginner_tutorials
Programming Assignment: ROS Services, Logging and Launch files
<p align="left">
<a href='https://opensource.org/licenses/MIT'><img src='https://img.shields.io/badge/License-MIT-brightgreen.svg'/></a>
</p>

## Project Description
This project uses ROS Kinetic Version with C++11 features to create a package containing talker and listener nodes. The talker is a server node and the user can use command-line to request a service to modify string message being printed by server. The client may also modify the default loop rate argument (= 10) to a new loop rate. Five types of logger messages are printed based on string or argument, provided as input from the user. A launch file is created that launches both talker and listener nodes simultaneously. 

## Assumptions 
1. ROS Kinetic Version is used for this project.
2. catkin is a build for ROS that is used for this project.

## Build Steps 
Please execute the following commands to build the package:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/likhitam/beginner_tutorials.git
cd ..
catkin_make

```
## Running Steps
Please execute the following steps after building the package to run it:
1. Open two terminals.
2. Make sure that a roscore is up and running.
3. In the first terminal execute the following commands: 

```
cd ~/catkin_ws
catkin_make
source ./devel/setup.bash
roscore

```
4. Now let us launch both talker and listener nodes simultaneously. In the second terminal execute the following commands to provide a loop rate of 20:

```
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch beginner_tutorials quickRun.launch freq:=20

```
Change looprate to other values by replacing 20.
5. To use the service to modify the text message execute the following commands in a new terminal:
```
rosservice call /modifyText "Say something nice."
```
6. The talker and listener nodes will continuously run. To end the process enter CTRL+C in both three terminals.

