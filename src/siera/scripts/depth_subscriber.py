#!/usr/bin/env python
import rospy
import sys
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2


class depth_synchroniser(object):
    """docstring for depth_synchroniser"""
    def __init__(self):
        super(depth_synchroniser, self).__init__()
        self.bridge = CvBridge()
        self.depth_pub = rospy.Publisher("/sync_depth_image", Image, queue_size=1)
        self.depth_sub = rospy.Subscriber("/D435I/depth/image_rect_raw", Image, self.callback)
        self.cache = []

    def callback(self,data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data)
            self.cache.append(depth_image)
            if len(self.cache)>=20:
                self.cache.pop(0)
            try:
                self.depth_pub.publish(self.bridge.cv2_to_imgmsg(self.cache[0]))
            except CvBridgeError as e:
                print(e)
        except CvBridgeError as e:
            print(e)

        

def convert_depth_image(ros_image):
    bridge = CvBridge()
     # Use cv_bridge() to convert the ROS image to OpenCV format
    try:
     #Convert the depth image using the default passthrough encoding
        depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="32FC1")
        cache.append(depth_image)
        depth_array = np.array(depth_image, dtype=np.float32)
        center_idx = np.array(depth_array.shape) / 2
        # print ('center depth:', depth_array[center_idx[0], center_idx[1]])

    except CvBridgeError, e:
        print e
     #Convert the depth image to a Numpy array
    # cv2.imshow("Image Window Depth", depth_image)
    if len(cache)>=20:
        cache.pop(0)
    cv2.imshow("Image Window Depth", cache[0])
    cv2.waitKey(3)


def pixel2depth():
	rospy.init_node('pixel2depth',anonymous=True)
	rospy.Subscriber("/D435I/depth/image_rect_raw", Image,callback=convert_depth_image, queue_size=1)
	rospy.spin()
    # cv.destroyAllWindows()

def main(args):
    ds = depth_synchroniser()
    rospy.init_node('depth_frame_synchroniser', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    # cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
    