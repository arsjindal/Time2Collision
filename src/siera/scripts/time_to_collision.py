#!/usr/bin/env python

import rospy
import sys
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math

class Time2Collision(object):
	"""docstring for Time2Collision
	This code intends to find final time to collision parameter by calculating velocity of robot at given time 
	and then using minimum distance from pedestrian to find time to collision
	"""
	def __init__(self):
		super(Time2Collision, self).__init__()
		self.bridge = CvBridge()
		# topic for publishing time to collision
		self.time2collision_pub = rospy.Publisher("/time2collision", String, queue_size=1)
		# subscribing to pedestrian minimum distance
		self.min_dist = rospy.Subscriber("/min_distance", String, self.callback_min_dist)
		# sunscribing to odom for find position
		self.odom_sub = rospy.Subscriber("/T265/odom/sample", Odometry, self.callback_odom)
		self.u = 0
		self.v = 0
		self.time = 0
		self.prev_pos_z = 0
		self.prev_pos_x = 0
		self.dist = 10


	def callback_min_dist(self, data):
		self.dist = float(data.data)

	def callback_odom(self, data):
		# finding time from time stamps
		current_time = data.header.stamp.secs + data.header.stamp.nsecs*(1e-9)
		# finding position of robot in x and z
		z_pos = data.pose.pose.position.z
		x_pos = data.pose.pose.position.x

		if self.time!=0:
			dt = current_time - self.time
		else:
			dt = 0
		self.time = current_time
		# finding net dispacement of robot 
		displacement = math.sqrt((z_pos-self.prev_pos_z)**2+(x_pos-self.prev_pos_x)**2)
		dz = z_pos - self.prev_pos_z
		if dt!=0:
			self.u = displacement/dt
		if abs(self.u)<0.05:
			self.u = 0
		if dz < 0:
			self.u = -1*self.u
		
		if abs(self.v- self.u) > 2:
			self.u = self.v
		# finding time to collision t = S/u
		if self.u != 0:
			self.time2collision = self.dist/self.u
		else:
			self.time2collision = "inf"
		try:
			if self.time2collision == "inf":
				self.time2collision_pub.publish(self.time2collision)
			else:
				self.time2collision_pub.publish(str(self.time2collision))
		except:
			print("Difficult to publish")
		self.prev_pos_z = z_pos
		self.prev_pos_x = x_pos
		self.v = self.u

def main(args):
	t2c = Time2Collision()
	rospy.init_node('time2collision', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting Down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)