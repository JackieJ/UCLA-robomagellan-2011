#include "ros/ros.h"
#include "glados/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}

int main(int argc, char** argv)
{
	const char* node_name = "MotorChatterboxNode";
	ros::init(argc, argv, node_name);


	// Command line parsing here, nothing to do, we're gonna try to KISS


	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<glados::AddTwoInts>("add_two_ints");
	glados::AddTwoInts srv;
	srv.request.a = 1;
	srv.request.b = 2;
	client.call(srv


	// now what, make the server
	ros::ServiceServer service = n.advertiseService("add_two_ints", add);
	ROS_INFO("Ready to add two ints.");
	ros::spin();

}
