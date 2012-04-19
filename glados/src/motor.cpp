#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "AX3500.h"
#include "glados/stateEstimation.h"
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "std_msgs/Float64.h"
#include "glados/odometry.h"
//#include <math.h>

#define CPR_ENCODER_VAL 10000

using namespace std;

const double PI = std::atan(1.0)*4;

class gladosMotor{
  
  unsigned int refreshRate;		
  AX3500  ax3500;
  ros::Publisher odometry_pub;
  float wheelbase;
  float wheelDiameter;
  //float PI;
public:
  
  unsigned int ticks_per_sec;		
  void setMotorSpeed(const geometry_msgs::Twist::ConstPtr& msg);
  void refresh();
  void initMotorController();
  void Close();
  gladosMotor(int&, char**);
};

gladosMotor::gladosMotor(int& argc, char** argv){
  
  ros::init(argc,argv,"motor_node");
  ros::NodeHandle n;
  
  odometry_pub = n.advertise<glados::odometry>("/odometry", 1000);
  
  wheelbase = 1.2;//m
  wheelDiameter = .35;//m
  //PI = 3.1415;
  //ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, &gladosMotor::setMotorSpeed, &motorNodeH);
  //ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, &gladosMotor::setMotorSpeed, this);
}

void gladosMotor::setMotorSpeed(const geometry_msgs::Twist::ConstPtr& msg)
{
  //typecasting
  char linear  = (char)(((msg->linear).x)*100);
  char angular = (char)(((msg->angular).x)*100);
  
  //setting speed
  ax3500.SetSpeed(AX3500::CHANNEL_LINEAR, -angular);
  ax3500.SetSpeed(AX3500::CHANNEL_STEERING, -linear);
}

void gladosMotor::refresh()
{
  //cout<<"starting refresh"<<endl;
  int l_ticks, r_ticks;
  ax3500.ReadEncoder(AX3500::ENCODER_1, AX3500::ABSOLUTE, l_ticks);
  ax3500.ReadEncoder(AX3500::ENCODER_2, AX3500::ABSOLUTE, r_ticks);
  
  glados::odometry msg;
  msg.left = (float)(wheelDiameter * PI * l_ticks / CPR_ENCODER_VAL);
  msg.right = (float)(wheelDiameter * PI * r_ticks / CPR_ENCODER_VAL);
  msg.heading = (float)((((msg.left - msg.right) / wheelbase) * PI) * 360);
  
  //cout<<"l encoder"<<l_msg.data<<endl;
  //ROS_INFO("wheel position %f,%f", l_msg.data,r_msg.data);
  odometry_pub.publish(msg);
}

void gladosMotor::initMotorController()
{
  bool isOpen = false;
  //string USB_SERIAL_PORTS[4] = {"/dev/ttyUSB0","/dev/ttyUSB1","/dev/ttyUSB2","/dev/ttyUSB3"}; 
  string USB_SERIAL_PORTS[1] = {"/dev/ttyUSB0"}; 
  
  //port connection sanity check
  int serial_port_counter = 0;
  while (!isOpen) {
    
    cout<<"connecting to serial port..."<<endl;
    
    isOpen = ax3500.Open(USB_SERIAL_PORTS[serial_port_counter], true);
    serial_port_counter++;
    
    if (serial_port_counter == 5) serial_port_counter = 0;
    cout<<"looping again"<<endl;
  }
  
  cout << "Resetting encoders";
  ax3500.ResetEncoder(AX3500::ENCODER_BOTH);
    
}
void gladosMotor::Close()
{
  ax3500.Close();
}

int main(int argc, char **argv)
{
  
  gladosMotor motorNodeH(argc,argv);
  
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, &gladosMotor::setMotorSpeed, &motorNodeH);
  
  int refreshRate = 10;
  ros::Rate r(refreshRate); // 10 hz
  
  motorNodeH.initMotorController();

  cout << "pre while";
//  while (ros::ok())
  while (true) {
    //		cout << "start while";
    motorNodeH.refresh();
    //    
    //... do some work ...
    ros::spinOnce();
    //		cout << "spun once";
    r.sleep();
  }
  motorNodeH.Close();
  return 0;
}
