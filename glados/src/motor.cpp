#include "ros/ros.h"
#include "AX3500.h"
#include "glados/stateEstimation.h"
#include <string>
#include <stdlib.h>
#include <stdio.h>
using namespace std;

//testing
//#include "glados/testMotorMsg.h"
#include "geometry_msgs/Twist.h"

AX3500  ax3500;

//testing
void pathReceiverFake(const geometry_msgs::Twist::ConstPtr& msg);
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
  }
  
  //ros::NodeHandle n;
  
  //cout<<"receiving twist msg"<<endl;
  
  //testing
  ax3500.SetSpeed(AX3500::CHANNEL_LINEAR, 5);
  ax3500.SetSpeed(AX3500::CHANNEL_STEERING, -5);
  
  
  
  //ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, pathReceiverFake); 
  //ros::Subscriber sub = n.subscribe("glados/stateEstimation", 1000, pathReceiver);
  
  cout << "Type \"q\" to quit";
  
  int count = 0;
  string str;
  getline(cin, str);
  
  cout << "Resetting encoder 1";
  ax3500.ResetEncoder(AX3500::ENCODER_BOTH);
  
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
  
  //cout<<"twist msg received"<<endl;
  //ros::spin();
  
  ax3500.Close();
  return 0;
  
}

/*
void pathReceiverFake(const geometry_msgs::Twist::ConstPtr& msg) {
  
  //typecasting
  char linear = (char)(((msg->linear).x)*70);
  char angular = (char)(((msg->angular).x)*70);
  
  //setting speed
  ax3500.SetSpeed(AX3500::CHANNEL_1, -angular);
  ax3500.SetSpeed(AX3500::CHANNEL_2, -linear);

  outputMotorData();
  
}

void outputMotorData() {
  
  char motorCon1Encoder;
  char motorCon2Encoder;
  ax3500.ReadSpeed(motorCon1Encoder,motorCon2Encoder);
  cout<<"motorCon1:"<<(int)motorCon1Encoder<<endl;
  cout<<"motorCon2:"<<(int)motorCon2Encoder<<endl;
  
}
*/

