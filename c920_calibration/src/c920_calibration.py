#!/usr/bin/env python

from __future__ import division
import rospy
import numpy as np
import cv2
import tf
from cv_bridge import CvBridge 

from sensor_msgs.msg import Image
from std_msgs.msg import String as StringMsg
np.seterr(all='raise')

class C920_Calibration:
	def __init__(self):
		self.bridge = CvBridge()
		im_sub =rospy.Subscriber('camera', Image, self.calibration)
		self.pub = rospy.Publisher('c920_calibration_image', Image, queue_size=1)  
	
	def calibration(self, image_data):
		if(image_data.encoding != 'bgr8'):
			raise ValueError('image is not bgr8 as expected')

		# convert both images to numpy arrays
		frame = self.bridge.imgmsg_to_cv2(image_data, desired_encoding='bgr8')	
		if(np.shape(frame)[0:2] != (480, 640)):
			raise ValueError('image does not have the right shape. shape(frame): {}, shape parameters:{}'.format(np.shape(frame)[0:2], (480, 640)))

		# draw rectangle to calibration
		frame = cv2.rectangle(frame,(305,320),(335,350),(255,0,0),3)
		image_msg = self.bridge.cv2_to_imgmsg(frame,"bgr8")
		self.pub.publish(image_msg)
	

if __name__ == '__main__':
	rospy.init_node('c920_calibration')
	c920_calibration=C920_Calibration()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.logwarn('failed')



