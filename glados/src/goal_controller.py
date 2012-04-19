#!/usr/bin/python
# written by asa
# simple pid controller to get to a setpoint of some kind.
import time
import sys

from gps import *

import roslib; roslib.load_manifest('geometry_msgs')
import rospy
from  geometry_msgs.msg import *
from  glados.msg import *


class pid(object):
    def __init__(self):
        self.last_value = 0.
        self.err = 0.
        self.Kp = .1
        self.Kd = .1
        self.Ki = .1
        self.output = 0.
        
    def calc(self, current):
        self.err = current - self.last_value
        self.output = current + self.err * self.Kd # add in diff and integral terms here....
        self.last_value = self.output
                
# http://cse.unl.edu/~carrick/courses/2011/496/lab2/lab2.html

class goalControl():
	def __init__(self):
		rospy.init_node('pid')
        self.goal_pos = [0.,0.]
        self.goal_angle = [0.]
        
        self.vel_pub = rospy.Publisher('cmd_vel', geometryMessages.Twist)
        
        self.sub = rospy.Subscriber("odometry", glados.Odometry, self.subscription_handler)
        
        self.pid = pid()
        self.current = 0.
        
    def subscription_handler(data):
        #testing============================
        self.current = data.heading
        
	#testing============================
        
    def run(self):
        while not rospy.is_shutdown():
            self.pid.calc(self.current)
            self.publish(self.pid.output)
            if self.rate:
                rospy.sleep(1/self.rate)
            else:
                rospy.sleep(1.0)
                
    def publish(self):
        self.vel_pub.publish(self.current_fix)

if __name__ == '__main__':
	g = goalControl()
	try:
            g.run()
	except rospy.ROSInterruptException: pass
	

