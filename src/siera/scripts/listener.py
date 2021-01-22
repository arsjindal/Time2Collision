#!/usr/bin/env python

from __future__ import print_function
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from imutils.object_detection import non_max_suppression
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class image_converter(object):
	"""docstring for image_converter
	This code is intended for reading raw image, pedestrian detection and finally displaying end result
	args: ttc(time to collision)
	"""
	def __init__(self,args):
		super(image_converter, self).__init__()
		# for publishing detected person coordinates
		self.coordinates_publisher = rospy.Publisher("/person_coordinates", String, queue_size=1)
		# for subscribing to raw image
		self.image_sub = rospy.Subscriber("/D435I/color/image_raw", Image, self.callback)
		# for subscribing to final time to collision output from time_to_collision.py
		self.time2collision_sub = rospy.Subscriber("/time2collision", String, self.callback_t2c)

		self.bridge = CvBridge()

		# Hog descriptor for person detection
		self.hog = cv2.HOGDescriptor()
		self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

		self.time2collision = 'inf'
		self.collision_threshold = int(args[1])
		self.warning = "No Pedestrians Under Threat"

	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			cv_image = cv2.GaussianBlur(cv_image,(5,5),0)
		except CvBridgeError as e:
			print(e)
		# rows, columns, channels = cv_image.shape
		# converting image to gray scale 
		cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		# pedestrian detection
		(regions, _) = self.hog.detectMultiScale(cv_image_gray, winStride=(4, 4), padding=(4, 4), scale=1.05)
		# filerting out good coordinated
		rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in regions])
		pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
		for (xA, yA, xB, yB) in pick:
			cv2.rectangle(cv_image, (xA,yA), (xB, yB), (0,0,255), 2)

		if self.warning!="No Pedestrians Under Threat":
			color = (0,0,255)
			scale = 0.75
		else:
			color = (0,255,0)
			scale = 1.0

		# Inking warning and threshold on main image
		cv2.putText(cv_image, self.warning, (50,50), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA) 
		cv2.putText(cv_image, "Thresh: %s"%str(self.collision_threshold), (50,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.LINE_AA) 
		
			# self.crop_image = cv_image[yA:yB, xA:xB]
		# cv2.imshow("Depth Image", self.depth_image)
		self.coordinates = self.process_coordinates(pick)
		try:
			self.coordinates_publisher.publish(self.coordinates)
		except:
			print("cannot publish")
		cv2.imshow("Image Window", cv_image)
		# cv2.imshow("crop_image", self.crop_image)
		cv2.waitKey(3)

	def callback_t2c(self, data):
		# collecting data from time to collision node
		self.time2collision = data.data
		if abs(float(self.time2collision)) < self.collision_threshold:
			print("Time to Collision < Threshold! Slow Down!!")
			self.warning = "Time to Collision < Threshold! Slow Down!!"
		else:
			self.warning = "No Pedestrians Under Threat"
		# print(self.time2collision)

	def process_coordinates(self,pick):
		string = ""
		for (xA, yA, xB, yB) in pick:
			if string=="":
				string = str(xA)+","+str(yA)+","+str(xB)+","+str(yB)
			else:
				string = string+";"+str(xA)+","+str(yA)+","+str(xB)+","+str(yB)
		return string

	def callback_depth(self, data):
		try:
			self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

		except CvBridgeError as e:
			print(e)

		# cv2.imshow("Depth Image", depth_image)
		# cv2.waitKey(3)

def main(args):
	# print(args)
	ic = image_converter(args)
	rospy.init_node('person_detection', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting Down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
