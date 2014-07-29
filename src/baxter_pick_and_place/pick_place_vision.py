#!/usr/bin/env python

"""
Description:

Node that performs primary vision operations for pick and place operation including:
	- Camera stream thresholding
	- Calculates image moments to locate objects
	- Publishes necessary information for other nodes

Help from: opencv-srf.blogspot.ro/2010/09/object-detection-using-color-seperation.html

"""



import roslib
import rospy

import sys

import cv
import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

import baxter_interface

from jon_baxter.srv import Coordinate


class Pick_Place_Vision(object):

	def __init__(self):
		"""
		Initialize Pick_Place_Vision class
		"""
		# Instantiate distance
		self.distance = 0.0

		# Instantiate all three cameras, close them, open left camera
		self._left_camera = baxter_interface.CameraController('left_hand_camera')
		self._right_camera = baxter_interface.CameraController('right_hand_camera')
		self._head_camera = baxter_interface.CameraController('head_camera')
		self.close_cameras()
		self.open_camera('left')

		# Open CV windows (resizable and positioned)
		cv2.namedWindow("Control", cv2.WINDOW_NORMAL)
		cv2.moveWindow("Control", 1990, 630)
		cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
		cv2.moveWindow("Original", 1990, 30)
		cv2.namedWindow("Thresholded", cv2.WINDOW_NORMAL)
		cv2.moveWindow("Thresholded", 3270, 30)

		# Initialize thresholding values
		low_h  = 165
		high_h = 179
		low_s  = 70
		high_s = 255
		low_v  = 60
		high_v = 255

		# "Do nothing" callback
		def nothing(x):
			pass

		# Create HSV trackbars (nothing on callback)
		cv2.createTrackbar("Low H", "Control", low_h, 179, nothing)
		cv2.createTrackbar("High H", "Control", high_h, 179, nothing)
		cv2.createTrackbar("Low S", "Control", low_s, 255, nothing)
		cv2.createTrackbar("High S", "Control", high_s, 255, nothing)
		cv2.createTrackbar("Low V", "Control", low_v, 255, nothing)
		cv2.createTrackbar("High V", "Control", high_v, 255, nothing)

		# Initialize ROS bridge
		self.bridge = CvBridge()

		# Initialize object locator publisher
		self.object_location_pub = rospy.Publisher("/camera_manager/object_location", Point)
		return


	def set_available(self):
		"""
		Sets Mover node to available
		"""
		# Try setting Mover node to available
		try:
			coordinate = rospy.ServiceProxy('coordinate', Coordinate)
			resp = coordinate(
				              'pick_place_vision', #node
			                  0,                   #status_request
			                  1,                   #status_set
			                  1)                   #status_setting

			# Return success (1 or 0)
			return resp.successful

		# Service exception
		except rospy.ServiceException, e:
			print "==[VISION]== Service call failed: %s" %e
			#Return unsuccessful
			return 0


	def set_busy(self):
		"""
		Sets Mover node to busy
		"""
		# Try setting Mover node to busy
		try:
			coordinate = rospy.ServiceProxy('coordinate', Coordinate)
			resp = coordinate(
				              'pick_place_vision', #node
			                  0,                   #status_request
			                  1,                   #status_set
			                  0)                   #status_setting

			# Return success (1 or 0)
			return resp.successful

		# Service exception
		except rospy.ServiceException, e:
			print "==[VISION]== Service call failed: %s" %e
			# Return unsuccessful 
			return 0


	def check_availability(self, node):
		"""
		Checks if another node is available
		"""
		# Try checking node availability
		try:
			coordinate = rospy.ServiceProxy('coordinate', Coordinate)
			resp = coordinate(
				              node,    #node
			                  1,       #status_request
			                  0,       #status_set
			                  0)       #status_setting

			# If successful, return availability (1 or 0)
			if resp.successful:
				return resp.available

		# Service exception
		except rospy.ServiceException, e:
			print "==[VISION]== Service call failed: %s" %e
			# Returns unsuccessful
			return 0


	def calibrate_distance(self):
		"""
		Retrieves the distance from Baxter's arm
		to the table and uses the value as a constant
		throughout the rest of the demo
		"""
		# Set node to busy
		self.set_busy()

		# Get left hand range state
		dist = baxter_interface.analog_io.AnalogIO('left_hand_range').state()

		# If > 65,000, did not get correct range
		if dist > 65000:
			sys.exit("==[VISION]== ERROR - calibrate_distance - no distance found")

		# Set value
		self.distance = dist

		# Set node to available
		self.set_available()


	def close_cameras(self):
		"""
		Close all three cameras
		"""
		# Close cameras
		self._left_camera.close()
		self._right_camera.close()
		self._head_camera.close()


	def open_camera(self, camera):
		"""
		Open a camera at max resolution
		"""
		# Open camera at resolution of 1280 x 800
		switch = {'left': self._left_camera,
		          'right': self._right_camera,
		          'head': self._head_camera}
		switch[camera].open()
		switch[camera].resolution = [1280, 800]


	def stream_images(self):
		"""
		Stream ROS images from Baxter's camera
		"""
		# Subscibe to Baxter's camera images
		_camera_sub = rospy.Subscriber("/cameras/left_hand_camera/image", Image, self._on_camera)


	def _on_camera(self, data):
		"""
		Camera Image callback: Converts ROS Image to HSV format,
		thresholds it, performs morphological opening and closing,
		and shows both the original and thresholded images in a
		window. Also publishes necessary information for
		pick_place_controller node
		"""
		# Convert Image message to CV image with blue-green-red color order (bgr8)
		try:
			img_original = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError, e:
			print("==[CAMERA MANAGER]==", e)

		# Convert image to HSV format
		img_hsv = cv2.cvtColor(img_original, cv2.COLOR_BGR2HSV)

		# Threshold image based on trackbar values
		low_h  = cv2.getTrackbarPos("Low H", "Control")
		high_h = cv2.getTrackbarPos("High H", "Control")
		low_s  = cv2.getTrackbarPos("Low S", "Control")
		high_s = cv2.getTrackbarPos("High S", "Control")
		low_v  = cv2.getTrackbarPos("Low V", "Control")
		high_v = cv2.getTrackbarPos("High V", "Control")
		img_thresholded = cv2.inRange(img_hsv, np.array([low_h, low_s, low_v]), np.array([high_h, high_s, high_v]))

		# Morphological opening (remove small objects from the foreground)
		img_thresholded = cv2.erode(img_thresholded, np.ones((2, 2), np.uint8), iterations=1)
		img_thresholded = cv2.dilate(img_thresholded, np.ones((2, 2), np.uint8), iterations=1)

		# Morphological closing (fill small holes in the foreground)
		img_thresholded = cv2.dilate(img_thresholded, np.ones((2, 2), np.uint8), iterations=1)
		img_thresholded = cv2.erode(img_thresholded, np.ones((2, 2), np.uint8), iterations=1)

		# Calculate the moments of the thresholded image
		moments = cv2.moments(img_thresholded)
		d_m01   = moments["m01"]
		d_m10   = moments["m10"]
		d_area  = moments["m00"]

		# If the area <= 10000, just noise
		if d_area > 10000:
			size  = img_hsv.shape
			pos_x = size[0] - (d_m01 / d_area)
			pos_y = size[1] - (d_m10 / d_area)

			# Publish object location
			self.object_location_pub.publish(Point(pos_x, pos_y, 0.000))


		# Show the CV image and wait 3ms for a keypress
		cv2.imshow("Original", img_original)
		cv2.imshow("Thresholded", img_thresholded)
		cv2.waitKey(3)




def main():
	print("==[VISION]== Initializing Vision")
	rospy.init_node('pick_place_vision')
	rospy.wait_for_service('coordinate')
	wait_rate = rospy.Rate(1)

	print("==[VISION]== Starting Vision...")
	ppv = Pick_Place_Vision()
	while not ppv.check_availability('pick_place_mover'):
		wait_rate.sleep()

	print("==[VISION]== Calibrating Vision")
	ppv.calibrate_distance()

	print("==[VISION]== Streaming images")
	ppv.stream_images()


	while not rospy.is_shutdown():
		rospy.spin()
	cv2.destroyAllWindows()
	print("\n==[VISION]== Vision done")




if __name__ == '__main__':
	main()