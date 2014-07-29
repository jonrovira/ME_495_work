#!/usr/bin/env python

"""
Description: 

Axes used:
(0, 0, 0): Center of Baxter's torso
+x: Protruding forward from Baxter
+y: Protruding to Baxter's left
+z: Protruding up from Baxter

"""

import operator
import numpy as np
import sys

import roslib
import rospy

import baxter_interface
from baxter_interface import CHECK_VERSION

from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import Point, Pose
from baxter_core_msgs.msg import EndpointState
from sensor_msgs.msg import CameraInfo

from jon_baxter.srv import Coordinate


class Pick_Place_Controller(object):

	def __init__(self):
		"""
		Initializes Pick_Place_Controller object
		"""
		# Initialize Baxter gripper, reboot, calibrate
		self.left_gripper = baxter_interface.Gripper('left', CHECK_VERSION)
		self.left_gripper.reboot()
		self.left_gripper.calibrate()

		# table height (get beforehnd)
		self.table_height = -0.320

		# Initialize pose query publisher
		self.pose_query_pub = rospy.Publisher("/pick_place_controller/pose_query", Pose)

		# Subscribe to Baxter's end-effector pose
		self._pose_sub = rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, self.set_current_pose)

		# Subscribe to object location from Camera Manager node
		self._object_location_sub = rospy.Subscriber("/camera_manager/object_location", Point, self.set_object_location)

		# Subscribe to image size from Camera Manager node
		self._image_siz_sub = rospy.Subscriber("/cameras/left_hand_camera/camera_info", CameraInfo, self.set_image_size)
		return


	def set_available(self):
		"""
		Sets Pick Place Controller node to available
		"""
		# Try setting Mover node to available
		try:
			coordinate = rospy.ServiceProxy('coordinate', Coordinate)
			resp = coordinate(
				              'pick_place_controller', #node
			                  0,       #status_request
			                  1,       #status_set
			                  1)       #status_setting

			# Return success (1 or 0)
			return resp.successful

		# Service exception
		except rospy.ServiceException, e:
			print "==[PICK PLACE CONTROLLER]== Service call failed: %s" %e
			#Return unsuccessful
			return 0


	def set_busy(self):
		"""
		Sets Pick Place Controller node to busy
		"""
		# Try setting Mover node to busy
		try:
			coordinate = rospy.ServiceProxy('coordinate', Coordinate)
			resp = coordinate(
				              'pick_place_controller', #node
			                  0,       #status_request
			                  1,       #status_set
			                  0)       #status_setting

			# Return success (1 or 0)
			return resp.successful

		# Service exception
		except rospy.ServiceException, e:
			print "==[PICK PLACE CONTROLLER]== Service call failed: %s" %e
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
			print "==[PICK PLACE CONTROLLER]== Service call failed: %s" %e
			# Returns unsuccessful
			return 0


	def set_current_pose(self, data):
		"""
		Sets/updates pose variable from range subscriber
		"""
		self.current_pose = data.pose
		return


	def set_object_location(self, data):
		"""
		Sets/updates object location variable from object location subscriber
		"""
		self.object_location = data
		return


	def set_image_size(self, data):
		"""
		Sets/updates image size variable from image size subscriber
		"""
		self.image_size = [data.height, data.width]
		return


	def wait_for_mover(self):
		"""
		Blocks until mover is available for commands
		"""
		r = rospy.Rate(1)
		while not self.mover_status == self.MOVER_AVAILABLE:
			r.sleep()
		return


	def get_workspace_coordinates(self):
		"""
		Takes image pixel coordinates subscribed to from the Camera Manager
		node and converts them to workspace coordinates using the following
		equation:

			B = (Pp - Cp) * cc * d + Bp + Go

		Where: 

		B  - Object in Baxter's space
		Pp - pixel coordinates
		Cp - center pixel coordinates
		Bp - Baxter pose
		Go - Gripper offset
		cc - camera calibration factor
		d  - distance from table
		"""
		# Set node to busy
		self.set_busy()

		# Get necessary values
		obj_loc = self.object_location
		img_size = self.image_size
		curr_pose = self.current_pose.position
		range_z = self.current_pose.position.z - self.table_height

		# Store values for calculation
		#          x               y            z
		Pp = [  obj_loc.x,     obj_loc.y,   obj_loc.z   ]
		Cp = [img_size[0]/2, img_size[1]/2,     0       ]
		Bp = [ curr_pose.x,   curr_pose.y,  curr_pose.z ]
		Go = [  -0.0230,        0.0110,      0.1100     ]
		cc = [   0.0021,        0.0021,      0.0000     ]
		d =  [   range_z,       range_z,     0.0000     ]

		# B = (Pp - Cp) * cc * d + Bp + Go
		Pp_minus_Cp = map(operator.sub, Pp, Cp)
		camera_constant = map(operator.mul, cc, d)
		pixel_to_real = map(operator.mul, Pp_minus_Cp, camera_constant)
		pixel_to_real[2] = range_z * -1
		B_without_gripper = map(operator.add, pixel_to_real, Bp)
		B = map(operator.add, B_without_gripper, Go)

		return B


	def query_vertical_pose(self, coordinates):
		"""
		Creates a Pose with a vertical, downward-facing end-effector
		orientation and a position matching the passed coordinates
		and publishes the pose as a query
		"""
		# Create pose query
		pose = Pose()
		pose.orientation.x = 0.0
		pose.orientation.y = 1.0
		pose.orientation.z = 0.0
		pose.orientation.w = 0.0
		pose.position.x = coordinates[0]
		pose.position.y = coordinates[1]
		pose.position.z = coordinates[2]

		# Publish pose query
		self.pose_query_pub.publish(pose)

		# Set node to available
		self.set_available()
		return


	def close_gripper(self):
		"""
		Close Baxter's left gripper
		"""
		# Set node to busy
		self.set_busy()

		# Grab cube
		self.left_gripper.close()
		rospy.sleep(1)

		# Set node to available
		self.set_available()


	def open_gripper(self):
		"""
		Open Baxter's left gripper
		"""
		# Set node to busy
		self.set_busy()

		# Open gripper
		self.left_gripper.open()
		rospy.sleep(1)

		# Set node to available
		self.set_available()





def main():
	print("==[PICK PLACE CONTROLLER]== Initializing node")
	rospy.init_node('pick_place_controller')
	rospy.wait_for_service('coordinate')
	wait_rate = rospy.Rate(1)

	print("==[PICK PLACE CONTROLLER]== Starting Pick Place Controller...")
	ppc = Pick_Place_Controller()
	while not ppc.check_availability('pick_place_vision'):
		wait_rate.sleep()

	print("==[PICK PLACE CONTROLLER]== Converting pixels to workspace coordinates and querying pose")
	object_coordinates = ppc.get_workspace_coordinates() #still not done
	ppc.query_vertical_pose(object_coordinates)
	while not ppc.check_availability('pick_place_mover'):
		wait_rate.sleep()

	print("==[PICK PLACE CONTROLLER]== Converting pixels to workspace coordinates and querying position")
	object_coordinates = ppc.get_workspace_coordinates() #not done doing work
	ppc.query_vertical_pose(object_coordinates)
	while not ppc.check_availability('pick_place_mover'):
		wait_rate.sleep()

	print("==[PICK PLACE CONTROLLER]== Grabbing cube")
	ppc.close_gripper()
	while not ppc.check_availability('pick_place_mover'):
		wait_rate.sleep()

	print("==[PICK PLACE CONTROLLER]== Dropping cube")
	ppc.open_gripper()

	while not rospy.is_shutdown():
		rospy.spin()
	print("\n==[PICK PLACE CONTROLLER]== Pick Place Controller done.")


if __name__ == '__main__':
	main()