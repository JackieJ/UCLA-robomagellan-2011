/* 
  Forward Bumper
  Aamoy Gupta - aamoyg@linux.ucla.edu
  Ed Solis - edsolis1@engineering.ucla.edu
  Description: Array of buttons is connected to pin 7 and gnd; this
               program indicates if bumper button(s) is/are pushed
               and publishes bumper state to appropriate rostopic
  Not Testing: roscore
  Todo: rosrun rosserial_python serial_node.py /devttyUSB1
    ^-  current handling this process is at /devttyUSB1
        [make sure IDE settings are correct before upload]
  Todo: rostopic echo pushed
 */

#include <ros.h>
#include <std_msgs/Bool.h>


ros::NodeHandle nh;

std_msgs::Bool bumper_msg;
ros::Publisher pub_bumper("bumped", &bumper_msg);

const int bumper_pin = 7;
const int led_pin = 13;

bool last_reading;
long last_debounce_time=0;
long debounce_delay=50;
bool bumper_published = true;

void setup()
{
  nh.initNode();
  nh.advertise(pub_bumper);
  
  //initialize an LED output pin 
  //and a input pin for our push button
  pinMode(led_pin, OUTPUT);
  pinMode(bumper_pin, INPUT);
  
  //Enable the pullup resistor on the button
  PORTD |= (1<<PD7);
  
  //The button is a normally button
  last_reading = ! digitalRead(bumper_pin);
 
}

void loop()
{
  
  bool reading = ! digitalRead(bumper_pin);
  
  if (last_reading!= reading){
      last_debounce_time = millis();
      bumper_published = false;
  }
  
  // if the button value has not changed for the debounce delay,
  // we know it's stable
  if ( !bumper_published && (millis() - last_debounce_time)
  	> debounce_delay)
  {
    digitalWrite(led_pin, reading);
    bumper_msg.data = reading;
    pub_bumper.publish(&bumper_msg);
    bumper_published = true;
  }

  last_reading = reading;
  
  nh.spinOnce();
}
