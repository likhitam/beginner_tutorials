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
 *  @file    test.cpp
 *  @author  Likhita Madiraju
 *  @date    11/13/2018
 *
 *  @brief Programming Assignment: ROS Publisher/Subscriber, Week 11, ROS Beginner Tutorial
 *
 *  @section DESCRIPTION
 *
 * Learning to use ROS Kinetic to create a test suite for talker node. 
 * Created test cases for talker node.   
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ros/service_client.h>
#include "std_msgs/String.h"
#include "beginner_tutorials/modifyText.h"

/**
 * @brief Testing whether the ROS service provided by talker node is present.
 */
TEST(TestNode, testServicePresent) {
  ros::NodeHandle n;
  // Client node created to check if the modifyText service is present
  ros::ServiceClient client =
      n.serviceClient<beginner_tutorials::modifyText>("modifyText");
  // Test whether the talker node service is present
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5.0)));
}

/**
 * @brief This test verifies whether the service by talker node, 
 * modifies the default text.
 */
TEST(TestNode, testServiceOutput) {
  ros::NodeHandle n;
  // Client node created to use the modifyText service
  ros::ServiceClient client =
      n.serviceClient<beginner_tutorials::modifyText>("modifyText");
  beginner_tutorials::modifyText::Request req;
  beginner_tutorials::modifyText::Response resp;

  req.newString = "This is a test for modifyText service.";
  std::string expectedString = req.newString;
  client.call(req, resp);
  EXPECT_EQ(expectedString, resp.responseString);
}

