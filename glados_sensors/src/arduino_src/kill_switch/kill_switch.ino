/*
  Kill Switch
  Aamoy Gupta - aamoyg@linux.ucla.edu
  Ed Solis - edsolis1@engineering.ucla.edu
  Description: The dead-man's switch is connected to pin 5 and gnd;
               this program indicates if bumper button(s) is/are
               pushed and publishes bumper state to appropriate
               rostopic
  Testing: roscore
  rosrun rosserial_python serial_node.py /devttyUSB1
    ^-  current handling this process is at /devttyUSB1
        [make sure IDE settings are correct before upload]
  rostopic echo dead
*/

#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

std_msgs::Bool death_msg;
ros::Publisher pub_death("dead", &death_msg);

const int death_pin = 5;
const int led_pin = 13;

bool death_last_reading;
long death_last_debounce_time=0;
long debounce_delay=50;
bool death_published = true;

void setup()
{
  nh.initNode();
  nh.advertise(pub_death);

  //initialize an LED output pin 
  //and a input pin for our push button
  pinMode(led_pin, OUTPUT);
  pinMode(death_pin, INPUT);

  //Enable the pullup resistor on the button
  PORTD |= (1<<PD5);

  //The button is a normally button
  death_last_reading = ! digitalRead(death_pin);
}

void loop()
{
  
  bool death_reading = ! digitalRead(death_pin);

  if (death_last_reading!= death_reading){
      death_last_debounce_time = millis();
      death_published = false;
  }
  
  // if the button value has not changed for the debounce delay,
  // we know it's stable
  if ( !death_published && (millis() - death_last_debounce_time)
  	> debounce_delay)
  {
    digitalWrite(led_pin, death_reading);
    death_msg.data = death_reading;
    pub_death.publish(&death_msg);
    death_published = true;
  }

  death_last_reading = death_reading;

  nh.spinOnce();
}

