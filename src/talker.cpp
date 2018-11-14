/************************************************************************
 MIT License
 Copyright (c) 2018 Likhita Madiraju
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 *************************************************************************/

/**
 *  @copyright MIT License 2018 Likhita Madiraju
 *  @file    talker.cpp
 *  @author  Likhita Madiraju
 *  @date    11/06/2018
 *
 *  @brief Programming Assignment: ROS Publisher/Subscriber, Week 10, ROS Beginner Tutorial
 *
 *  @section DESCRIPTION
 *
 * Learning to use ROS Kinetic to provide a service for publisher node.
 * 1. Using a publisher node, talker and added a service to modify the text message. 
 * 2. Added five types of logger messages between two nodes to display useful messages.
 * 3. Added code for receiving a command-line argument to modify the loop rate.
 */

#include <log4cxx/logger.h>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/modifyText.h"
#include "tf/transform_broadcaster.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

extern std::string m = "Default message. Enter your own message. ";

/**
 *   @brief Service for modifying the text message from default.
 *
 *   @param request: service request by client to modify text
 *   @param response: response to request by server
 *
 *   @return bool
 */
bool modifyText(beginner_tutorials::modifyText::Request& request,
                beginner_tutorials::modifyText::Response& response) {
  m = request.newString;
  return true;
}

int main(int argc, char **argv) {
  // Setting logger level to display debug message
  log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(
      ros::console::g_level_lookup[ros::console::levels::Debug]);
  ros::console::notifyLoggerLevelsChanged();
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  ros::ServiceServer server = n.advertiseService("modifyText", modifyText);
  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  double freq = 10;
  // Verify if argument is passed
  if (argc == 2) {
    freq = atoi(argv[1]);
    ROS_WARN_STREAM("Loop rate has been changed to " << freq);
    // Check if a valid argument has been passed
    if (freq <= 0) {
      ROS_FATAL_STREAM("Please enter a frequency greater than zero.");
      return -1;
    }
  }
  ROS_DEBUG_STREAM("Set frequency = " << freq);
  ros::Rate loop_rate(freq);
  
  static tf::TransformBroadcaster br;
  // Transform object created called transform
  tf::Transform transform;
  // Set origin of the talk frame with respect to world frame
  transform.setOrigin(tf::Vector3(10.0, 6.0, -2.0));
  tf::Quaternion q;
  q.setRPY(0.5, 0.8, 1.57);
  // Set rotation of the talk frame with respect to world frame
  transform.setRotation(q);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    if (m == "") {
      ROS_ERROR_STREAM("Message not entered. Re-run with new string.");
    return -1;
    }
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;
    std::stringstream ss;
    ss << m << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);
    // Broadcasting transformation of talk frame to world frame
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                     "world", "talk"));
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}

