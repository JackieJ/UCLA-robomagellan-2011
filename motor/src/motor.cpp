/*
 * Copyright (C) 2011-2112 Garrett Brown <gbruin@ucla.edu>
 *
 * This Program is free software; you can redistribute it and/or modify it
 * under the terms of the Modified BSD License.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *    3. Neither the name of the organization nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * This Program is distributed AS IS in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

/*
 * This code is created from the ROS tutorial "Writing a Simple Publisher and Subscriber (C++)"
 * at <http://ros.org/wiki/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29>.
 * For a breakdown of each line, check out the Subscriber part of that tutorial.
 */

// Include all the headers necessary to use the most common public pieces of the ROS system
#include "ros/ros.h"
// Include the turtlesim/Velocity message from the turtlesim package
//#include "turtlesim/Velocity.h"
// This lets us interface with the AX3500 motor controller
#include "AX3500.h"
//#include "std_msgs/String.h"
#include "glados/stateEstimation.h"
AX3500 ax3500;

void ReceiveVelocity(const glados::stateEstimation::ConstPtr& msg)
{
  //ROS_INFO("velocity: [%s]", msg->data.c_str());
  int linear = (int)(msg->linear * 5);
  int angular = (int)(msg->angular * 5);

  // Note, the channels are reversed and inverted because motor 2 is rotated 180*
  ax3500.SetSpeed(AX3500::CHANNEL_LINEAR, -angular);
  ax3500.SetSpeed(AX3500::CHANNEL_STEERING, -linear);
}

/*
void SensorBasedNavigator(const sensor::integratedMsg::ConstPtr& msg) {
  
}
*/



int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_node");
  
  std::cout << "Connecting to GLaDOS motor controller...\n";
  ax3500.Open("/dev/ttyUSB0", true); // Enable safety cutoff
  
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("glados/stateEstimation", 1000, ReceiveVelocity);
  ros::spin();
  
  ax3500.Close();
  return 0;
}
