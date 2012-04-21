#!/usr/bin/python
# written by asa to interface with gpsd
# I didn't follow this tutorial here: http://www.ros.org/wiki/gpsd_client/Tutorials/Writing%20a%20Subscriber%20for%20gpsd_client%20%28C%2B%2B%29
#import threading
import time
import gps
import sys

from gps import *

import roslib; roslib.load_manifest('gps_common')
import rospy
from  gps_common.msg import *

class GpsClient():
	def __init__(self):
		rospy.init_node('gpsd_client')
		#threading.Thread.__init__(self)
		self.session = gps()
		self.session.stream(WATCH_ENABLE|WATCH_NEWSTYLE)
		self.current_value = None
		self.fixpub = rospy.Publisher('gps/fix', GPSFix)
		self.statuspub = rospy.Publisher('gps/status', GPSStatus)
		self.current_fix = GPSFix()
		
	def get_current_value(self):
		return self.current_value
	
	def run(self):
		while not rospy.is_shutdown():
			try:
				while True:
					self.current_value = self.session.next()
					print >> sys.stderr, self.get_current_value()
					if 'lat' in self.current_value and 'lon' in self.current_value:
						self.publish()
			except StopIteration:
				pass
	
	def publish(self):
		#try:
			#self.current_value['lat']
			#self.current_fix.latitude = self.current_value['lat']
			#self.current_fix.longitude = self.current_value['lon']
		#except Exception as err:
		#	print >>sys.stderr, err
		self.fixpub.publish(self.current_fix)
		self.statuspub.publish(GPSStatus(self.current_status))
		pass

if __name__ == '__main__':
	gpsp = GpsClient()
	try:
		gpsp.run()
	except rospy.ROSInterruptException: pass
	
