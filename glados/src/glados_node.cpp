#include "ros/ros.h"
//#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "glados/MotorStatusStamped.h"

#include "AX3500.h"


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
