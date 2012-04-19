/*
  Battery monitor
*/

// Calibration!!!!!
// When Vout = 5V (AnalogIn = 1023):
Vout1 = 7.2
Vout2 = 7.2

// Pins to read voltage
const int batt1_pin = 6;
const int batt2_pin = 4;

#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

std_msgs::Bool batt1_msg;
std_msgs::Bool batt2_msg;
ros::Publisher pub_batt1("Volt1", &batt1_msg);
ros::Publisher pub_batt2("Volt2", &batt2_msg);

void setup()
{
  nh.initNode();
  nh.advertise(pub_death);

  //initialize an LED output pin 
  //and a input pin for our push button
  pinMode(batt1_pin, OUTPUT);
  pinMode(batt2_pin, INPUT);
}

void loop()
{

  batt1_msg.data = analogRead(batt1_pin);
  batt2_msg.data = analogRead(batt2_pin);
  pub_batt1.publish(&batt1_msg);
  pub_batt2.publish(&batt2_msg);

  nh.spinOnce();
}

