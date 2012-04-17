#include "ros/ros.h"
#include "AX3500.h"
#include "glados/stateEstimation.h"
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <std_msgs/Float64.h>

using namespace std;

//testing
//#include "glados/testMotorMsg.h"
#include "geometry_msgs/Twist.h"

AX3500  ax3500;

//testing
void setMotorSpeed(const geometry_msgs::Twist::ConstPtr& msg);
void outputMotorData();

//void pathReceiver(const glados::stateEstimation::ConstPtr& msg);
//void publishMotorMsg();

string USB_SERIAL_PORTS[3] = {"/dev/ttyUSB0","/dev/ttyUSB1","/dev/ttyUSB2"};

int main(int argc, char **argv) {
  ros::init(argc, argv, "motor_node");
  
    bool isOpen = false;

  //port connection sanity check
  int serial_port_counter = 0;
  while (!isOpen) {
    
    cout<<"connecting to serial port..."<<endl;
    
    isOpen = ax3500.Open(USB_SERIAL_PORTS[serial_port_counter], false);
    serial_port_counter++;
    
    if (serial_port_counter == 4) serial_port_counter = 0;
	cout<<"looping again"<<endl;
}
  
  ros::NodeHandle n;
  
  cout<<"receiving twist msg"<<endl;
  
  //testing
 /* ax3500.SetSpeed(AX3500::CHANNEL_LINEAR, 7);
  ax3500.SetSpeed(AX3500::CHANNEL_STEERING, -7);
  char encoder1_val;
  char encoder2_val;
  while(true) {
    for(time_t t = time(0) + 2 ; time(0) < t;) {}
    ax3500.ReadSpeed(encoder1_val,encoder2_val);
    for(int i = 0; i < 8 ; i++) {
      //bitwise shift;
    }
  }
  return 0;
 */
  ros::Subscriber sub = n.subscribe("/cmd_vel", 1000,               setMotorSpeed); 
// ros::Subscriber sub = n.subscribe("glados/stateEstimation", 1000, pathReceiver);
  
  ros::Publisher odometry_pub = n.advertise<std_msgs::Float64>("/odometry", 1000);

  cout << "Type \"q\" to quit";
  
  int count = 0;
  string str;
  getline(cin, str);
  
  cout << "Resetting encoder 1";
  ax3500.ResetEncoder(AX3500::ENCODER_BOTH);
  /*
  while (true)
  {
    // wait for the wheel to revolve
    string str;
    getline(cin, str);
    
    int ticks;
    ax3500.ReadEncoder(AX3500::ENCODER_1, AX3500::ABSOLUTE, ticks);
    count++;
    double avg = ticks / count;
    
    cout << "Average CPR: " << avg;
    
    if (str == "q")
    {
      cout << endl;
      break;
    }
  }
  */
ros::Rate r(10); // 10 hz
int l_ticks,r_ticks;

while (ros::ok())
{
  ax3500.ReadEncoder(AX3500::ENCODER_1, AX3500::ABSOLUTE, l_ticks);
  ax3500.ReadEncoder(AX3500::ENCODER_2, AX3500::ABSOLUTE, r_ticks);
   
  std_msgs::Float64 l_msg; 
  l_msg.data = (l_ticks / 10000. ) * .35 * 3.1415;
  ROS_INFO("wheel position %f", l_msg.data); 
  odometry_pub.publish(l_msg);

  //	
  //... do some work ...
  r.sleep();

}


  //cout<<"twist msg received"<<endl;
  //ros::spin();
  
  ax3500.Close();
  return 0;
  
}


void setMotorSpeed(const geometry_msgs::Twist::ConstPtr& msg) {
  
  //typecasting
  char linear = (char)(((msg->linear).x)*70);
  char angular = (char)(((msg->angular).x)*70);
  cout<<"msg_ recieved:"<<linear<<endl;
  
  //setting speed
  ax3500.SetSpeed(AX3500::CHANNEL_LINEAR, -angular);
  ax3500.SetSpeed(AX3500::CHANNEL_STEERING, -linear);

  outputMotorData();
  
}

void outputMotorData() {
  
  char motorCon1Encoder;
  char motorCon2Encoder;
  ax3500.ReadSpeed(motorCon1Encoder,motorCon2Encoder);
  cout<<"motorCon1:"<<(int)motorCon1Encoder<<endl;
  cout<<"motorCon2:"<<(int)motorCon2Encoder<<endl;
  
}


