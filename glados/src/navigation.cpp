/*
 * Copyright (C) 2112 Garrett Brown <gbruin@ucla.edu>
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

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

// Heads towards LS 5926 (target coords unused for now)
double target_lat = 34.0673;
double target_log = -118.4432;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "navigation_node");

	ros::NodeHandle n;

	// publish a geometry_msgs/Twist on the cmd_vel topic
	ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	// create new geometry_msgs::Twist that holds linear and angular velocities
	geometry_msgs::Twist command;
	command.linear.x = 0; // m/s
	command.linear.y = 0;
	command.linear.z = 0;
	command.angular.x = 0; // rad/s
	command.angular.y = 0;
	command.angular.z = 0;

	tf::TransformListener listener;

	// wait for tf to begin receiving this information about this transform from the odometry node
	listener.waitForTransform("base_footprint", "odom_combined", ros::Time(0), ros::Duration(1, 0));

	// record the starting transform from the odometry to the base frame
	tf::StampedTransform start_transform;
	tf::StampedTransform current_transform;
	listener.lookupTransform("base_footprint", "odom_combined", ros::Time(0), start_transform);

	// publish twist_msg until node is killed
	ros::Rate loop_rate(10); // Hz
	while (n.ok())
	{
		vel_pub.publish(command);
		ros::spinOnce();
		loop_rate.sleep();

		try
		{
			listener.lookupTransform("base_footprint", "odom_combined", ros::Time(0), current_transform);
		}
		catch (const tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			break;
		}

		// See how far we've traveled
		tf::Transform relative_transform = start_transform.inverse() * current_transform;
		double dist_moved = relative_transform.getOrigin().length();

	}

	// Grind the robot to a halt
	command.linear.x = command.linear.y = command.linear.z = 0;
	command.angular.x = command.angular.y = command.angular.z = 0;
	vel_pub.publish(command);

	return 0;
}
