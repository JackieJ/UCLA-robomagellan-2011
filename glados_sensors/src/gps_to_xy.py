#!/usr/bin/env python
import roslib; roslib.load_manifest('glados_sensors')
import rospy
from std_msgs.msg import 

def callback(data):
    rospy.loginfo(rospy.get_name()+"I heard %s",data.data)

def convertGPS():
    rospy.init_node('gps_to_xy', anonymous=True)
    rospy.Subscriber("xy", , callback)
    rospy.spin()

if __name__ == '__main__':
    convertGPS()
