#!/usr/bin/env python

import rospy
import sys
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2
from std_msgs.msg import String


class Depth_Calculator(object):
	"""docstring for Depth_Calculator
	This code is intended to calculate min distance from pedestrian to robot using pedestrian coordinates and depth image
	"""
	def __init__(self):
		super(Depth_Calculator, self).__init__()
		self.bridge = CvBridge()
		# self.arg = arg
		# Publisher topic for pedestrian minimum distance
		self.min_distance = rospy.Publisher("/min_distance", String, queue_size=1)
		# subscribing to pedestrian coordinated from listener.py
		self.coordinates_sub = rospy.Subscriber("/person_coordinates", String, self.callback_coord)
		# subscribing to depth image
		# self.depth_sub = rospy.Subscriber("/sync_depth_image", Image, self.callback)
		self.depth_sub = rospy.Subscriber("/D435I/depth/image_rect_raw", Image, self.callback)

	def callback_coord(self, data):
		coords = data.data.split(";")
		self.coords = []
		for coord in coords:
			self.coords.append(coord.split(","))


	def callback(self, data):
		try:
			self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
			self.depth_array = np.array(self.depth_image, dtype=np.float32)
			self.cropped_image = []
			self.min = np.mean(self.depth_array)
			# print(self.min)
			areas = []
			# filtering out coordinates
			for coords in self.coords:
				# print(coords)
				if coords!=['']:
					xA = int(coords[0])
					xB = int(coords[2])
					yA = int(coords[1])
					yB = int(coords[3])
					# using only 40% of pedestrian coordinates to find calculate depth from actual pedestrian body
					x1 = int((xB-xA)/2 - 0.2*(xB-xA))
					x2 = int((xB-xA)/2 + 0.2*(xB-xA))
					y1 = int((yB-yA)/2 - 0.2*(yB-yA))
					y2 = int((yB-yA)/2 + 0.2*(yB-yA))
					cv2.rectangle(self.depth_image, (int(coords[0]),int(coords[1])), (int(coords[2]), int(coords[3])), (0,0,255), 2)
					areas.append(abs((yB-yA)*(xB-xA)))
					self.cropped_image.append(self.depth_array[y1:y2,x1:x2])

			# print(np.array(self.cropped_image[0]).min(), "min")
			# areas = []
			try:
				# finding index to max area. Pedestrian with max bbox area is closest to robot 
				max_idx = areas.index(max(areas))
				arr = self.cropped_image[max_idx]
				arr[arr==0] = np.nan
				minimum = np.nanmean(arr)
				if minimum < self.min:
					self.min = minimum
			except:
				pass

			try:
				self.min_distance.publish(str(self.min/1000))
			except:
				print("error in publishing")

		except Exception as e:
			pass

		# cv2.imshow("Depth Image", self.depth_image)
		# cv2.waitKey(3)

def main(args):
	dc = Depth_Calculator()
	rospy.init_node('Depth_Calculator', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting Down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)