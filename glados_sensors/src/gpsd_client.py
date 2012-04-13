#!/usr/bin/python
# written by asa to interface with gpsd

#import threading
import time
import gps

from gps import *

import roslib; roslib.load_manifest('gps_common')
import rospy
from  gps_common.msg import *

class GpsClient():
	def __init__(self):
		rospy.init_node('gpsd_client')
		#threading.Thread.__init__(self)
		self.session = gps(mode=WATCH_ENABLE)
		self.current_value = None
		self.fixpub = rospy.Publisher('gps/fix', GPSFix)
		self.statuspub = rospy.Publisher('gps/status', GPSStatus)
		self.current_fix = GPSFix()
		self.rate = 10.
	def get_current_value(self):
		return self.current_value
	
	def run(self):
		while not rospy.is_shutdown():
			try:
				while True:
					self.current_value = self.session.next()
                			#time.sleep(0.2) # tune this, you might not get values that quicklya
					self.publish
					if self.rate:
						rospy.sleep(1/self.rate)
					else:
						rospy.sleep(1.0)
			except StopIteration:
				pass
	
	def publish(self):
		self.current_fix.latitude = self.current_value['latitude']
		#self.fixpub.publish(GPSFix(self.current_value)
		#self.statuspub.publish(GPSStatus(self.current_status))
		pass

if __name__ == '__main__':
	gpsp = GpsClient()
	try:
		gpsp.run()
	except rospy.ROSInterruptException: pass
	
