/*
 * This code is created from the ROS tutorial "Writing a Simple Publisher and Subscriber (C++)"
 * at <http://ros.org/wiki/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29>.
 * For a breakdown of each line, check out the Subscriber part of that tutorial.
 */

// Include all the headers necessary to use the most common public pieces of the ROS system
#include "ros/ros.h"
// Include the turtlesim/Velocity message from the turtlesim package
#include "turtlesim/Velocity.h"
// This lets us interface with the AX3500 motor controller
#include "AX3500.h"


AX3500 ax3500;

void ReceiveVelocity(const turtlesim::Velocity::ConstPtr& msg)
{
	int linear = (int)(msg->linear * 5);
	int angular = (int)(msg->angular * 5);

	// Note, the channels are reversed and inverted because motor 2 is rotated 180*
	ax3500.SetSpeed(AX3500::CHANNEL_LINEAR, -angular);
	ax3500.SetSpeed(AX3500::CHANNEL_STEERING, -linear);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "glados_node");

	std::cout << "Connecting to GLaDOS motor controller...\n";
	ax3500.Open("/dev/ttyUSB0");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("turtle1/command_velocity", 1000, ReceiveVelocity);
	ros::spin();

	ax3500.Close();
	return 0;
}
