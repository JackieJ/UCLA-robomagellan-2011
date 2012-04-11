#!/usr/bin/python
# written by asa to interface with gpsd

import threading
import time
import gps

from gps import *

import roslib; roslib.load_manifest('gps_common')
import rospy
from  gps_common.msg import *

class GpsClient(threading.Thread):
	def __init__(self):
		rospy.init_node('gpsd_client')
		threading.Thread.__init__(self)
		self.session = gps(mode=WATCH_ENABLE)
		self.current_value = None
		self.fixpub = rospy.Publisher('gps/fix', GPSFix)
		self.statuspub = rospy.Publisher('gps/status', GPSStatus)

	def get_current_value(self):
		return self.current_value
	
	def run(self):
		try:
			while True:
				self.current_value = self.session.next()
                		#time.sleep(0.2) # tune this, you might not get values that quickly
		except StopIteration:
			pass
	def publish(self):
		#self.fixpub.publish(GPSFix(self.current_value)
		#self.statuspub.publish(GPSStatus(self.current_status))
		pass
if __name__ == '__main__':
	gpsp = GpsClient()
	gpsp.start()
	# gpsp now polls every .2 seconds for new data, storing it in self.current_value
	while 1:
	# In the main thread, every 5 seconds print the current value
		time.sleep(5)
		print gpsp.get_current_value().keys()
		gpsp.publish()
