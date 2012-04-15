#include "ros/ros.h"
#include "glados_sensors/imu.h"
#include <string>

using namespace std;

void messageProcessor(const glados_sensors::imu::ConstPtr& msg) {
  /*
  cout<<"latitude:"<<(msg->latitude)<<endl;
  cout<<"longitude:"<<(msg->longitude)<<endl;
  cout<<"altitude:"<<(msg->altitude)<<endl;
  cout<<"track:"<<(msg->track)<<endl;
  cout<<"speed:"<<(msg->speed)<<endl;
  cout<<"time:"<<(msg->time)<<endl;
  cout<<"covariance list:"<<endl;
  for(int index = 0 ; index<<(msg->position_covariance).size();index) {
    cout<<(msg->position_variance[index])<<endl;  
  }
  */
  cout<<"x-coordinate:"<<msg->gx<<endl;
  cout<<"y-coordinate:"<<msg->gy<<endl;
  cout<<"bearing:"<<msg->bearing<<endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "glados_node");
  cerr<< "listening to GPS msg..."<<endl;
  ros::NodeHandle n;
  //subscribe to gps fix
  ros::Subscriber sub = n.subscribe("/imu", 1000, messageProcessor);
  ros::spin();
}
