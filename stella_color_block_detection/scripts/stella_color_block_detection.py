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

class ColorBlockDetection:
	def __init__(self):
		self.bridge = CvBridge()
		self.targetUpper = np.array(rospy.get_param('~target/upper'))
		self.targetLower = np.array(rospy.get_param('~target/lower'))
		self.pictureHeight= rospy.get_param('~pictureDimensions/pictureHeight')
		self.pictureWidth = rospy.get_param('~pictureDimensions/pictureWidth')

		im_sub =rospy.Subscriber('camera_for_wlkata', Image, self.trackObject)
		self.pub = rospy.Publisher('color_block_detection_image', Image, queue_size=1)  
	
	def trackObject(self, image_data):
		if(image_data.encoding != 'bgr8'):
			raise ValueError('image is not bgr8 as expected')

		# convert both images to numpy arrays
		frame = self.bridge.imgmsg_to_cv2(image_data, desired_encoding='bgr8')	
		if(np.shape(frame)[0:2] != (self.pictureHeight, self.pictureWidth)):
			raise ValueError('image does not have the right shape. shape(frame): {}, shape parameters:{}'.format(np.shape(frame)[0:2], (self.pictureHeight, self.pictureWidth)))

		# convert to HSV color space
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)	

		# select all the pixels that are in the range specified by the target
		org_mask = cv2.inRange(hsv, self.targetUpper, self.targetLower)
	
		# clean that up a little, the iterations are pretty much arbitrary
		mask = cv2.erode(org_mask, None, iterations=4)
		mask = cv2.dilate(mask,None, iterations=3)

		# find contours of the object
		contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
			
		# go threw all the contours. starting with the bigest one
		for contour in sorted(contours, key=cv2.contourArea, reverse=True):
			centerRaw, size, rotation = cv2.minAreaRect(contour)
			center = np.round(centerRaw).astype(int)
			size = np.round(size).astype(int)

			if(max(size[0],size[1])/min(size[0],size[1]) > 1.2):
				image_msg = self.bridge.cv2_to_imgmsg(frame,"bgr8")
				self.pub.publish(image_msg)
				return

			size = (size[0]+size[1])/2

			if(size < 15):
				image_msg = self.bridge.cv2_to_imgmsg(frame,"bgr8")
				self.pub.publish(image_msg)
				return

			if(size < 19):size=19

			distance_x = 12/size
			distance_y = 0.52*distance_x*(center[0]-320)/320*-1
			distance_z = 0.39*distance_x*(center[1]-240)/240*-1
			rotation = int(rotation) * -1

			br = tf.TransformBroadcaster()
			br.sendTransform((distance_x, distance_y, distance_z),
					tf.transformations.quaternion_from_euler(0, 0, np.radians(rotation)),
                      			rospy.Time.now(),
                      			"color_block",
                      			"camera_link")

			rect = cv2.minAreaRect(contour)
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			cv2.drawContours(frame,[box],-0,(0,255,0),3)
			image_msg = self.bridge.cv2_to_imgmsg(frame,"bgr8")
			self.pub.publish(image_msg)
			return
		image_msg = self.bridge.cv2_to_imgmsg(frame,"bgr8")
		self.pub.publish(image_msg)
	

if __name__ == '__main__':
	rospy.init_node('stella_coloe_block_detection')
	detection=ColorBlockDetection()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.logwarn('failed')



