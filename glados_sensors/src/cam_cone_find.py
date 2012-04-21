#!/usr/bin/env python
import roslib
roslib.load_manifest('glados_sensors')
import sys
import rospy
import cv
from std_msgs.msg import String,Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class cam_cone_finder:
	def __init__(self):
	#	self.image_pub = rospy.Publisher("image_topic_2",Image)
		self.cone_track_pub = rospy.Publisher("/cone/offset",Float32)
		self.cone_area_pub = rospy.Publisher("/cone/area",Float32)
		
		self.image_sub 		= rospy.Subscriber("/camera/image_raw",Image,self.callback)
	
		cv.NamedWindow("Image window", 1)
		self.bridge = CvBridge()
		self.debug = False
		
	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
		except CvBridgeError, e:
			print e
		
		(cols,rows) = cv.GetSize(cv_image)
		if cols > 60 and rows > 60 :
			cv.Circle(cv_image, (50,50), 10, 255)
		cv.Smooth(cv_image, cv_image, cv.CV_BLUR, 10); 
	
		#convert the image to hsv(Hue, Saturation, Value) so its  
		#easier to determine the color to track(hue) 
		hsv_img = cv.CreateImage(cv.GetSize(cv_image), 8, 3) 
		cv.CvtColor(cv_image, hsv_img, cv.CV_BGR2HSV) 
	
		#limit all pixels that don't match our criteria, in this case we are  
		#looking for purple but if you want you can adjust the first value in  
		#both turples which is the hue range(120,140).  OpenCV uses 0-180 as  
		#a hue range for the HSV color model 
		thresholded_img =  cv.CreateImage(cv.GetSize(hsv_img), 8, 1) 
		cv.InRangeS(hsv_img, (0 * 180/240, 80, 80), (18 * 180/240, 255, 255), thresholded_img) 
		cv.Erode(thresholded_img, thresholded_img, None, 10)
		cv.Dilate(thresholded_img, thresholded_img, None, 18)
		
		#determine the objects moments and check that the area is large  
		#enough to be our object 
		moments = cv.Moments(cv.GetMat(thresholded_img), 0) 
		area = cv.GetCentralMoment(moments, 0, 0) 
		
		#there can be noise in the video so ignore objects with small areas 
		if(area > 500000): 
			#determine the x and y coordinates of the center of the object 
			#we are tracking by dividing the 1, 0 and 0, 1 moments by the area 
			x = cv.GetSpatialMoment(moments, 1, 0)/area
			y = cv.GetSpatialMoment(moments, 0, 1)/area
			#print 'x: ' + str(x) + ' y: ' + str(y) + ' area: ' + str(area) 
			#create an overlay to mark the center of the tracked object 
			if self.debug:
				overlay = cv.CreateImage(cv.GetSize(cv_image), 8, 3) 
				cv.Circle(overlay, (int(x), int(y)), 2, (255, 255, 255), 20) 
				cv.Add(cv_image, overlay, cv_image) 
				#add the thresholded image back to the img so we can see what was  
				#left after it was applied 
				cv.Merge(thresholded_img, None, None, None, cv_image)
			scaled_x = ((float(x)/cols) - .5)*2 
#			print >> sys.stderr,scaled_x,area
			self.cone_track_pub.publish(scaled_x) 
			self.cone_area_pub.publish(area) 
			
		#display the image  
		if self.debug:
			cv.ShowImage("Image window", cv_image)
			cv.WaitKey(1)
	
	#	try:
	#		self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image, "bgr8"))
	#	except CvBridgeError, e:
	#		print e

def main(args):
	rospy.init_node('cam_cone_find', anonymous=True)
	ic = cam_cone_finder()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
	cv.DestroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
