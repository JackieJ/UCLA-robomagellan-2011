#!/usr/bin/python
# written by asa
# simple pid controller to get to a setpoint of some kind.
import time
import sys
import math
import yaml
from gps import *

import roslib; roslib.load_manifest('geometry_msgs')
import rospy
from  geometry_msgs.msg import *
import glados.msg

waypoints_list = yaml.load(file('../../waypoints.yaml','r'))

class pid(object):
    def __init__(self):
        self.last_value = 0.
        self.err_x = 0.
        self.err_y = 0.
        self.Kp = .5
        self.Kd = .1
        self.Ki = .1
        self.output = {
            'linear':{
                'x':0.,
                'y':0.,
                'z':0.
                },
            'angular':{
                'x':0.,
                'y':0.,
                'z':0.
                }
            }
        self.target_angle = 0.
        
    def calc(self, current_pos, goal_pos):
        if current_pos['getData'] == False:
            return
        '''
        #testing
        #x
        self.err_x = goal_pos['x'] - current_pos['x']
        x_v = self.err_x*self.Kp
        if x_v != 0 and math.fabs(self.err_x) < 0.5*current_pos['x']:
            x_v = self.err_x*(self.Kp+self.Ki)
        #y
        self.err_y = goal_pos['y'] - current_pos['y']
        y_v = self.err_y*self.Kp
        if y_v != 0 and math.fabs(self.err_y) < 0.5*current_pos['y']:
            y_v = self.err_y*(self.Kp+self.Ki)

        v = float(math.sqrt((x_v*x_v + y_v*y_v)))
        
        if self.err_y < 0.:
            self.output['linear']['x'] = -v
        else:
            self.output['linear']['x'] = v
        if v != 0.:
            other_v = self.err_y/v #y2-y1/v
            angular_offset = math.fabs(other_v)
            if self.err_x < 0.:
                self.output['angular']['z'] = angular_offset
            else:
                self.output['angular']['z'] = -angular_offset
                
        else:
            self.output['angular']['z'] = 0.
        '''
        #difference calculation
        self.err_x = goal_pos['x'] - current_pos['x']
        self.err_y = goal_pos['y'] - current_pos['y']
        distance = float(math.sqrt(self.err_x*self.err_x+self.err_y*self.err_y))
        self.target_angle = (90 - (math.atan2(self.err_y,self.err_x)*(180/math.pi))) - (current_pos['heading']*(180/math.pi))
        print self.target_angle
        
        self.output['angular']['z'] = (self.target_angle*(math.pi)/(180))
        print self.output['angular']['z']
        
        if distance != 0.:
            self.output['linear']['x'] = 0.
            
# http://cse.unl.edu/~carrick/courses/2011/496/lab2/lab2.html
class goalControl():
    def __init__(self):
        rospy.init_node('pid')
        #test value        
        self.goal_pos = [{'x':1.,'y':0.}]
                
        self.vel_pub = rospy.Publisher('cmd_vel', Twist)
        
        self.sub = rospy.Subscriber("odom", glados.msg.odometry, self.subscription_handler)
        
        #TODO:sub to gpsd and compass to get current pos and heading

        self.pid = pid()
        self.rate = 1.0
        
        self.current_pos = {'getData':True,'x':0.,'y':0.,'heading':-math.pi/2}
        self.current_twist = {
            'linear':{
                'x':0.,
                'y':0.,
                'z':0.
                },
            'angular':{
                'x':0.,
                'y':0.,
                'z':0.
                }
            }

    def subscription_handler(self,data):
        pass
        
    def run(self):
        while not rospy.is_shutdown() and self.goal_pos and self.current_pos['getData'] == True:
            self.pid.calc(self.current_pos, self.goal_pos[0])
            #increment current pos
            self.current_pos['heading'] += (self.pid.output['angular']['z'])*self.pid.Kp
            
            
            #record twist msg
            self.current_twist['linear']['x'] = self.pid.output['linear']['x']
            self.current_twist['angular']['z'] = self.pid.output['angular']['z']
            
            #debug output
            '''
            print "goal pos:"
            print self.goal_pos[0]
            print "current pos:"
            print self.current_pos
            print "current twist:"
            print self.current_twist
            '''
            
            #publish twist msg
            twistOutput = Twist()
            twistOutput.linear.x = self.current_twist['linear']['x']
            twistOutput.linear.y = 0.
            twistOutput.linear.z = 0.
            twistOutput.angular.x = 0.
            twistOutput.angular.y = 0.
            twistOutput.angular.z = -self.current_twist['angular']['z']
            
            #emergency kill
            #twistOutput.linear.x = 0.
            #twistOutput.angular.z = 0.
            
            self.publish(twistOutput)
            rospy.sleep(self.rate)
                
    def publish(self,output):
        self.vel_pub.publish(output)

if __name__ == '__main__':
    g = goalControl()
    try:
        g.run()
    except rospy.ROSInterruptException: pass
	

