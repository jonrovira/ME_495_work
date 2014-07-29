#!/usr/bin/python2

"""
Displays a given image file on Baxter's face.

Pass the relative or absolute file path to an image
file on your computer, and the example will read and
the example will read and convert the image using
cv_bridge, sending it to the screen as a standard ROS
Image message.
"""

import sys

import rospy

import cv
import cv_bridge

from sensor_msgs.msg import (
	Image,
)




def send_image(path):
	"""
	Send the image located at the specified path to the head
	display on Baxter.

	@param path: path to the image file to load and send
	"""
	img = cv.LoadImage(path)
	msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
	pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
	pub.publish(msg)
	# Sleep to allow for image to be published
	rospy.sleep(1)


def main():
	rospy.init_node('rsdk_xdisplay_image', anonymous=True)

	send_image('/home/jon-76/jons_stuff/ros/baxter_ws/src/baxter_tools/share/images/researchsdk.png')
	return 0


if __name__ == '__main__':
	sys.exit(main())