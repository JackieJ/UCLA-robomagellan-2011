#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

// Messages includes
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include "glados/odometry.h"
#include "glados/wheelspeed.h"

#include "AX3500.h"
#include <math.h>
//#include <stdlib.h>

#define TICKS_PER_REVOLUTION 10000

using namespace std;

#ifndef PI
  #define PI 3.14159265358979 // double precision gives 15-17 digits
#endif

class gladosMotor{
  AX3500 ax3500;
  ros::Publisher custom_odom_pub;
  ros::Publisher odom_pub;
  ros::Publisher wheelspeed_pub;
  const double wheelbase; // in m
  const double wheelDiameter; // in m

  double x;
  double y;
  double theta;
  double vx;
  double vy;
  double vtheta;
  ros::Time previousRefresh;

  // For Jackie's custom odom message
  double left_accumulated;
  double right_accumulated;

public:
  gladosMotor();
  ~gladosMotor();
  void setStationaryPose(double x, double y, double theta);
  void setMotorSpeed(const geometry_msgs::Twist::ConstPtr& msg);
  void refresh();
  void initMotorController();
};

gladosMotor::gladosMotor() : x(0), y(0), theta(0),
                             vx(0), vy(0), vtheta(0),
                             left_accumulated(0), right_accumulated(0),
                             wheelbase(1.2 /* m */), wheelDiameter(0.35 /* m */)
{
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, &gladosMotor::setMotorSpeed, this);
  custom_odom_pub = n.advertise<glados::odometry>("/odometry", 1000);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Publisher wheelspeed_pub = n.advertise<glados::wheelspeed>("wheelspeed", 50);

  
  previousRefresh = ros::Time::now();
  // Sleep for 1ms to avoid a divide by zero when calling refresh()
  ros::Duration(0.001).sleep();

  // Open the motor controller
  bool isOpen = false;
  const char* USB_SERIAL_PORTS[4] = {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"};

  // port connection sanity check
  int serial_port_counter = 0;
  while (!isOpen)
  {
    isOpen = ax3500.Open(USB_SERIAL_PORTS[serial_port_counter], true);
    serial_port_counter++;
    if (serial_port_counter == sizeof(USB_SERIAL_PORTS) / sizeof(USB_SERIAL_PORTS[0]))
    	serial_port_counter = 0;
  }

  ax3500.ResetEncoder(AX3500::ENCODER_BOTH);

}

gladosMotor::~gladosMotor()
{
  ax3500.Close();
}

void gladosMotor::setMotorSpeed(const geometry_msgs::Twist::ConstPtr& msg)
{
  // Convert Twist msg into the robot's base-local coordinate frame "base_footprint"
  // This is only necessary if the Twist message has a header_id other than base_footprint

  // Convert from m/s to char
  const double char_per_mps = 100;
  char linear  = (char)(msg->linear.x * char_per_mps);
  // use wheelbase/2 to convert from rad/s to m/s at the edge of the wheel
  char angular = (char)(msg->angular.z * (wheelbase/2) * char_per_mps);
  
  ax3500.SetSpeed(AX3500::CHANNEL_LINEAR, -angular);
  ax3500.SetSpeed(AX3500::CHANNEL_STEERING, -linear);
}

void gladosMotor::refresh()
{
  int l_ticks, r_ticks;
  // Use relative ticks, as it gives you ticks since last read
  ax3500.ReadEncoder(AX3500::ENCODER_1, AX3500::RELATIVE, l_ticks);
  ax3500.ReadEncoder(AX3500::ENCODER_2, AX3500::RELATIVE, r_ticks);

  // Calculate dt
  ros::Time now = ros::Time::now();
  double dt = (now - previousRefresh).toSec();
  
  double left = wheelDiameter * PI * l_ticks / TICKS_PER_REVOLUTION;
  double right = wheelDiameter * PI * r_ticks / TICKS_PER_REVOLUTION;

  // Jackie's custom odom message
  left_accumulated += left;
  right_accumulated += right;
  glados::odometry msg;
  msg.left = left_accumulated;
  msg.right = right_accumulated;
  msg.heading = (left_accumulated - right_accumulated) / wheelbase * PI * 360;
  custom_odom_pub.publish(msg);

  glados::wheelspeed msg2;
  msg2.left = l_ticks / dt;
  msg2.right = r_ticks / dt;
  wheelspeed_pub.publish(msg2);

  /*
  // First, publish the displacement of the coordinate frame translating with the robot
  // Second, publish the displacement of the robot in that coordinate frame

  // In the coordinate frame translating with the robot
  // Forward displacement is (left + right) / 2
  // Angular displacement is atan2(right - left, wheelbase)
  double old_x = x, old_y = y, old_theta = theta;
  x += (left + right) / 2 * cos(theta);
  y += (left + right) / 2 * sin(theta);
  theta += atan2(right - left, wheelbase);

  vx = (x - old_x) / dt;
  vy = (y - old_y) / dt;
  vtheta = (theta - old_theta) / dt;

  // Since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(theta);

  // First, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = now;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

//set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;

  //publish the message
  odom_pub.publish(odom);
  */

}

void gladosMotor::setStationaryPose(double new_x, double new_y, double new_theta)
{
	x = new_x;
	y = new_y;
	theta = new_theta;
	vx = 0;
	vy = 0;
	vtheta = 0;

	// Not used outside the custom odom message
	left_accumulated = 0;
	right_accumulated = 0;

	previousRefresh = ros::Time::now();
	// Sleep for 1ms to avoid a divide by zero when calling refresh()
	ros::Duration(0.001).sleep();
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"motor_node");

  gladosMotor motorNodeH;
  
  int refreshRate = 10; // Hz
  ros::Rate r(refreshRate);
  
  while (ros::ok()) {
    motorNodeH.refresh();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
