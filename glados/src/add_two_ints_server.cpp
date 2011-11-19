#include "ros/ros.h"
#include "glados/AddTwoInts.h"

bool add(glados::AddTwoInts::Request  &req,
         glados::AddTwoInts::Response &res )
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;


  return 0;
}
