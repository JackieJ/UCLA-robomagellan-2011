#include "ros/ros.h"
#include "glados/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    const char *node_name = "MotorNode";
    ros::init(argc, argv, node_name);

	// For listening
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Rate loop_rate(10);

	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback); // need a callback
	// done listening forever
	ros::spin();

	int count = 0;
	while (ros.ok())
	{
		// For talking
		ros::NodeHandle n;\
		ROS_INFO(node_name);//ROS_INFO("%s", msg.data.c_str());

	    chatter_pub.publish(msg);

	    ros::spinOnce();

	    loop_rate.sleep();
	    ++count;
	}
	return 0;
}


