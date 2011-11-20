#include "ros/ros.h"
#include "turtlesim/Velocity.h"

#include "AX3500.h"

AX3500 ax3500;

void ReceiveVelocity(const turtlesim::Velocity::ConstPtr& msg)
{
	int linear = (int)(msg->linear * 5/2);
	int angular = (int)(msg->angular * 5/2);

	// Notice, the channels are reversed because motor 2 is rotated around
	ax3500.SetSpeed(AX3500::CHANNEL_LINEAR, angular);
	ax3500.SetSpeed(AX3500::CHANNEL_STEERING, linear);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "glados_node");

	ax3500.Open("/dev/ttyUSB0");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("turtle1/command_velocity", 1000, ReceiveVelocity);

	ros::spin();

	ax3500.Close();

	return 0;
}
