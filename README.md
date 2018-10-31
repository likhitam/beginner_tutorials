# beginner_tutorials
Programming Assignment: ROS Publisher/Subscriber
<p align="left">
<a href='https://opensource.org/licenses/MIT'><img src='https://img.shields.io/badge/License-MIT-brightgreen.svg'/></a>
</p>

## Project Description
This project uses ROS Kinetic Version with C++11 features to create a package containing publisher/subscriber nodes. The publisher, "talker node", and subscriber, "listener node", are linked to a topic, "chatter". The talker node publishes a string message.  

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
1. Open three terminals.
2. Make sure that a roscore is up and running.
3. In the first terminal execute the following commands: 

```
cd ~/catkin_ws
catkin_make
source ./devel/setup.bash
roscore

```
4. Now let us run the talker node. In the second terminal execute the following commands:

```
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials talker

```
5. Next we must run the listener node to verify if it has subscribed to chatter. In the third terminal execute the following commands:

```
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials listener

```
6. The talker and listener nodes will continuously run. To end the process enter CTRL+C in all three terminals.
