#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "glados_node");

	ros::NodeHandle n;

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}
