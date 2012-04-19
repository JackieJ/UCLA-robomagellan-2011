#include <ros/ros.h>

// Message includes
#include <geometry_msgs/Twist.h>
#include "glados/wheelspeed.h"

#include <iostream>
#include <fstream>

using namespace std;

#define FILENAME "/home/garrett/Documents/ros/motor_test.csv"

class MotorTest
{
public:
	MotorTest();
	~MotorTest() { }

	void doTest(double speed);
	void receiveSpeed(const glados::wheelspeed::ConstPtr &speed);

private:
	glados::wheelspeed::ConstPtr getSpeed();

	ros::NodeHandle n;
	ros::Publisher vel_pub;
	ros::Subscriber speed_sub;
	glados::wheelspeed::ConstPtr speed_results;
};

MotorTest::MotorTest()
{
	vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	speed_sub = n.subscribe("wheelspeed", 1000, &MotorTest::receiveSpeed, this);

	// Open the file for writing
	ofstream myfile;
	myfile.open(FILENAME); // This should truncate the file, if not use ios_base::trunc
	if (myfile.is_open())
	{
		myfile << "ref_speed,left_speed,left_var,right_speed,right_var,avg_speed,avg_var" << endl;
		myfile.close();
	}
	else
	{
		cout << "Error opening file: " << FILENAME << endl;
	}
}

void MotorTest::receiveSpeed(const glados::wheelspeed::ConstPtr &speed)
{
	speed_results = speed;
}

glados::wheelspeed::ConstPtr MotorTest::getSpeed()
{
	return speed_results;
}

void MotorTest::doTest(double speed)
{
	if (ros::ok())
	{
		cout << "Running a speed test of " << speed << endl;
		// Do the test here
		geometry_msgs::Twist command;
		command.linear.x = speed; // m/s
		command.linear.y = 0;
		command.linear.z = 0;
		command.angular.x = 0; // rad/s
		command.angular.y = 0;
		command.angular.z = 0;
		vel_pub.publish(command);
		ros::spinOnce(); // Is this necessary to publish our message?
		// Give time for transients to settle
		ros::Duration(5).sleep();
		// Give enough time to fill the 10s buffer
		ros::Duration(2).sleep();
		ros::spinOnce(); // Purge the callback queue
		glados::wheelspeed::ConstPtr w = getSpeed();
		// Record the results
		ofstream myfile;
		myfile.open(FILENAME, ios::out | ios::app);
		myfile << speed << "," << w->left << "," << w->left_variance << ","
		                       << w->right << "," << w->right_variance << ","
		                       << w->avg << "," << w->avg_variance << endl;
		myfile.close();
		ros::spinOnce();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_motor_node");

	MotorTest test;

	test.doTest(0.1);
	test.doTest(0.2);
	test.doTest(0.3);
	test.doTest(0.4);
	test.doTest(0.5);
	test.doTest(0.6);
	test.doTest(0.7);
	test.doTest(0.8);
	test.doTest(0.9);
	test.doTest(1.0);
	test.doTest(1.1);
	test.doTest(1.2);
	test.doTest(1.3);
	test.doTest(1.4);
	test.doTest(1.5);
	test.doTest(1.6);
	test.doTest(1.7);
	test.doTest(1.8);
	test.doTest(1.9);
	test.doTest(2.0);
	test.doTest(2.1);
	test.doTest(2.2);
	test.doTest(2.3);
	test.doTest(2.4);
	test.doTest(2.5);
	test.doTest(2.6);
	test.doTest(2.7);
	test.doTest(2.8);

	return 0;
}
