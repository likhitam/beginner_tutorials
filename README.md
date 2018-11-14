# beginner_tutorials

Programming Assignment - ROS TF, unit testing, bag files

<p align="left">
<a href='https://opensource.org/licenses/MIT'><img src='https://img.shields.io/badge/License-MIT-brightgreen.svg'/></a>
</p>

## Project Description

This project uses ROS Kinetic Version with C++11 features to create a package containing talker and listener nodes. The talker is a server node and the user can use command-line to request a service to modify string message being printed by server. The talker node broadcasts a tf frame called /talk with parent /world and has non-zero translation and rotation. The client may also modify the default loop rate argument (= 10) to a new loop rate. Five types of logger messages are printed based on string or argument, provided as input from the user. A launch file is created that launches both talker and listener nodes simultaneously, with an feature to record talker published messages using rosbag. Used gtest/rostest to create two Level 2 integration tests, that test the talker node for the service it provides. 

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
## Unit Testing
Run the following commands to see the output of test cases written for talker node:
```
cd ~/catkin_ws
source ./devel/setup.bash
catkin_make run_tests_beginner_tutorials
rostest beginner_tutorials testNode.launch
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
Change loop rate to other values by replacing 20. When you change the loop rate you will receive a warning message. If you enter loop rate <= 0, you will recieve fatal message and node will terminate. Else if the loop rate is set succesfully, you will recieve debug message and info will print continuously.

5. Launch with enabling recording of the message generated on the topic chatter by running the following command:
```
roslaunch beginner_tutorials quickRun.launch store:=true
```
To launch without recording the messages execute the followig command:
```
roslaunch beginner_tutorials quickRun.launch store:=false
```
To play the recorded messages execute in the results folder the following command:
```
cd ~/catkin_ws/src/beginner_tutorials/results
rosbag play record.bag
```
6. To verify the TF frames using tf_echo and rqt_tf_tree, execute the following commands open three new terminals:
Terminal 1: broadcasted message will print
```
rosrun tf tf_echo /world /talk
```

Terminal 2: rqt tree will show the tf frames 
```
rosrun rqt_tf_tree rqt_tf_tree
```
Terminal 3: After verification run the following command to generate pdf of tf frames being broadcasted
```
rosrun tf view_frames
evince frames.pdf
```
7. To use the service to modify the text message execute the following commands in a new terminal:
```
rosservice call /modifyText "Say something nice."
```
If you enter new text message as "" empty, you will recieve error message and node will terminate.

8. The talker and listener nodes will continuously run. To end the process enter CTRL+C in both three terminals.


