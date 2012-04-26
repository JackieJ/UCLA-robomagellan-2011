#!/usr/bin/python
# written by asa
# simple pid controller to get to a setpoint of some kind.
import time
import sys
import math
import yaml
from gps import *
import roslib; roslib.load_manifest('geometry_msgs')
roslib.load_manifest('nav_msgs')
roslib.load_manifest('gps_common')
from gps_common.msg import *
import rospy
from  geometry_msgs.msg import *
from nav_msgs.msg import *
from glados.srv import *
roslib.load_manifest('glados_sensors')
from glados_sensors.msg import *
roslib.load_manifest('std_msgs')
from std_msgs.msg import *

waypoints_list = yaml.load(file('/home/glados/ros_workspace/UCLA-robomagellan/glados/src/waypoints.yaml','r'))


def LLtoUTMConverter(lat,lon):
    rospy.wait_for_service('LLtoUTM')
    try:
        lltoutm = rospy.ServiceProxy('LLtoUTM', LLtoUTM)
        #print lltoutm.__dict__.keys()
        #print lltoutm.request_class.__dict__.keys()
        #request = lltoutm.request_class()
        #response = lltoutm.response_class()
        response = lltoutm(lat,lon)
        return response
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        

class pid(object):
    def __init__(self):
        self.last_value = 0.
        self.err_x = 0.
        self.err_y = 0.
        self.Kp = .5
        self.Kd = .5
        self.Ki = .1
        self.linear_v = 1.
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
        self.mode = 'gotoWaypoint'
        self.cone_direction = '0'
        self.hasCone = False
        
        
    def calc(self, current_pos, goal_pos):
        if current_pos['getData'] == False:
            return
        #print current_pos
        #print goal_pos
        self.hasCone = goal_pos['hasCone']
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
        
        err_x = goal_pos['x'] - current_pos['x']
        if math.fabs(err_x) <= 0.01:
            self.err_x = 0
        else:
            self.err_x = err_x
        err_y = goal_pos['y'] - current_pos['y']
        if math.fabs(err_y) <= 0.01:
            self.err_y = 0
        else:
            self.err_y = err_y
        distance = float(math.sqrt(self.err_x*self.err_x+self.err_y*self.err_y))
        
        if distance < 6. and self.hasCone:
            self.mode == 'findCone'
        
        if self.mode == 'gotoWaypoint':
            self.target_angle = (math.atan2(self.err_y,self.err_x)*(180/math.pi)) # - (current_pos['current_heading']-current_pos['previous_heading'])
            
        elif self.mode == 'findCone':
            self.target_angle = self.cone_direction * 21.5
            
        #self.target_angle = current_pos['current_heading']-current_pos['previous_heading']
        angle_err = self.target_angle - current_pos['theta']
        
        print>>sys.stderr,self.mode,"target_angle",self.target_angle,"theta",current_pos['theta'],"angle_err", angle_err, "current_heading",current_pos['current_heading'],'x',current_pos['x'],'y',current_pos['y'],"err xy", self.err_x, self.err_y,"distance",distance
        
        
        self.output['angular']['z'] = (angle_err * self.Kp  * math.pi)/(180.)
        
#        self.output['angular']['z'] = (self.target_angle*(math.pi)/(180.))*self.Kp
        
        #print self.output['angular']['z']
        
        if math.fabs(self.target_angle) >= 90:
            self.output['linear']['x'] = 0
        
        if distance != 0. and distance >= 0.3:
            self.output['linear']['x'] = self.linear_v*self.Kd
        elif distance < 0.3 and distance != 0:
            self.output['linear']['x'] = self.linear_v*self.Kd*0.6
        elif distance == 0:
            self.output['linear']['x'] = 0
            self.output['angular']['z'] = 0
          
# http://cse.unl.edu/~carrick/courses/2011/496/lab2/lab2.html
class goalControl():
    def __init__(self):
        rospy.init_node('pid')
        #test value        
        self.goal_pos = waypoints_list
        self.current_goal_pos = {
            "hasCone":False,
            "x":5.,
            "y":0.
            }
        self.vel_pub = rospy.Publisher('cmd_vel', Twist)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_handler)
        self.gps_sub = rospy.Subscriber('gps/fix', GPSFix, self.gps_handler)
        self.imu_sub = rospy.Subscriber('imu', imu, self.imu_handler)
        self.cam_sub = rospy.Subscriber('/cone/offset', Float32, self.cam_handler)
        
        self.pid = pid()
        self.rate = 1.0
        self.go = True
        self.current_pos = {
            'getData':True,
            'x':0.,
            'y':0.,
            'current_heading':360,
            'previous_heading':360,
            'theta':0.0,
            'linear':{
                'x':0.,
                'y':0.,
                'z':0.
                },
            'angular':{
                'x':0.,
                'y':0.,
                'z':0.
                },
            'left_v':0.,
            'right_v':0.
            }
            
        self.output_pos = {
            'x':0.,
            'y':0.,
            'current_heading':360,
            'previous_heading':360,
            'linear':{
                'x':0.,
                'y':0.,
                'z':0.
                },
            'angular':{
                'x':0.,
                'y':0.,
                'z':0.
                },
            'left_v':0.,
            'right_v':0.
            }

    def odom_handler(self, data):
#        print >>sys.stderr,data
        self.current_pos['x'] = data.pose.pose.position.x
        self.current_pos['y'] = data.pose.pose.position.y
        self.current_pos['theta'] = data.twist.twist.angular.y
        
#        self.current_odom = data.data['pose']['pose']['position']
        
    def gps_handler(self, data):
        return
        response = LLtoUTMConverter(data.latitude, data.longitude)
        self.current_pos['x'] = response.easting
        self.current_pos['y'] = response.northing
        print>>sys.stderr,"current pos", response.easting,",",response.northing
    
    def imu_handler(self, data):
        return
        self.current_pos['previous_heading'] = self.current_pos['current_heading']
        self.current_pos['current_heading'] = data.bearing
        #print data
    
    def goNextWaypoint(self):
        if self.goal_pos:
            popout = self.goal_pos.pop(0)
            llutmresponse = LLtoUTMConverter(popout['latitude'], popout['longitude'])
#            self.current_goal_pos['x'] = llutmresponse.easting
#            self.current_goal_pos['y'] = llutmresponse.northing
            self.current_goal_pos['x'] = 5.0
            self.current_goal_pos['y'] = 1.0

            self.current_goal_pos['hasCone'] = popout['hasCone']
            self.go = True
        else:
            self.go = False
            
    def cam_handler(self):
        self.pid.cone_direction = msg.data
        
    def run(self):
        #print glados_sensors.__dict__.keys()
        #self.goNextWaypoint()
        while (not rospy.is_shutdown()):#and (self.current_pos['getData']) and self.go :
            #if (math.fabs(self.current_pos['x'] - self.current_goal_pos['x']) <= 0.01) and (math.fabs(self.current_pos['y'] - self.current_goal_pos['y']) <= 0.01):
                #self.goNextWaypoint()
                
            self.pid.calc(self.current_pos, self.current_goal_pos)
            #increment current pos
            #self.current_pos['heading'] += (self.pid.output['angular']['z'])*self.pid.Kp

            #record twist msg
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
            twistOutput.linear.x = self.pid.output['linear']['x']
            twistOutput.linear.y = 0.
            twistOutput.linear.z = 0.
            twistOutput.angular.x = 0.
            twistOutput.angular.y = 0.
            twistOutput.angular.z = -self.pid.output['angular']['z']
            
            #emergency kill
            #twistOutput.linear.x = 0.
            #twistOutput.angular.z = 0.
            
            self.publish(twistOutput)
            rospy.sleep(self.rate)
            
        #stop the robot once it's runing out
        twistOutput = Twist()
        twistOutput.linear.x = 0.
        twistOutput.linear.y = 0.
        twistOutput.linear.z = 0.
        twistOutput.angular.x = 0.
        twistOutput.angular.y = 0.
        twistOutput.angular.z = 0.
        self.publish(twistOutput)
                
    def publish(self,output):
        self.vel_pub.publish(output)
        
if __name__ == '__main__':
    '''latitude = waypoints_list[0]['latitude']
    longitude = waypoints_list[0]['longitude']
    response = LLtoUTMConverter(latitude, longitude)
    print response'''
    g = goalControl()
    try:
        g.run()
    except rospy.ROSInterruptException: pass
	

