#!/usr/bin/env python

import sys
import numpy as np
import operator

import rospy
import moveit_commander

import cv2
from cv_bridge import CvBridge, CvBridgeError

import baxter_interface
from   baxter_interface import CHECK_VERSION

from sensor_msgs.msg      import ( Image )
from geometry_msgs.msg    import ( Pose,
	                               PoseStamped,
	                               Point )
from  moveit_msgs.msg     import ( DisplayTrajectory,
	                               PositionIKRequest,
	                               CollisionObject,
	                               Grasp )
from shape_msgs.msg       import ( SolidPrimitive )
from baxter_core_msgs.msg import ( EndpointState )


class Pick_Place(object):

	def __init__(self):
		"""
		Initialize class

		"""
		# Overall MoveIt initialization
		moveit_commander.roscpp_initialize(sys.argv)
		self.robot = moveit_commander.RobotCommander()

		# MoveIt scene initialization
		self.scene = moveit_commander.PlanningSceneInterface()
		self.table_height = -0.320
		p = PoseStamped()
		p.header.frame_id = self.robot.get_planning_frame()
		p.pose.position.x =  0.8
		p.pose.position.y =  0.025
		p.pose.position.z = -0.6
		co = CollisionObject()
		co.operation = CollisionObject.ADD
		co.id = "table"
		co.header = p.header
		box = SolidPrimitive()
		box.type = SolidPrimitive.BOX
		box.dimensions = list((0.75, 1.25, 0.55))
		co.primitives = [box]
		co.primitive_poses = [p.pose]
		pub_co = rospy.Publisher('collision_object', CollisionObject, latch=True)
		pub_co.publish(co)

		# Moveit group initialization
		self.group = moveit_commander.MoveGroupCommander('left_arm')
		self.group.set_goal_position_tolerance(0.12)
		self.group.set_goal_orientation_tolerance(0.10)

		# Gripper initialization
		self.left_gripper = baxter_interface.Gripper('left', CHECK_VERSION)
		self.left_gripper.reboot()
		self.left_gripper.calibrate()

		# Camera initialization
		self.left_camera  = baxter_interface.CameraController('left_hand_camera')
		self.right_camera = baxter_interface.CameraController('right_hand_camera')
		self.head_camera  = baxter_interface.CameraController('head_camera')
		self.right_camera.close()
		self.head_camera.close()
		self.left_camera.open()
		self.left_camera.resolution = [1280, 800]
		self.left_camera_size = [800, 1280]
		_camera_sub = rospy.Subscriber('/cameras/left_hand_camera/image', Image, self._on_camera)

		# Open CV initialization
		cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
		cv2.moveWindow('Original', 1990, 30)
		cv2.namedWindow('Thresholded', cv2.WINDOW_NORMAL)
		cv2.moveWindow('Thresholded', 3270, 30)
		self.low_h  = 165
		self.high_h = 179
		self.low_s  = 70
		self.high_s = 255
		self.low_v  = 60
		self.high_v = 255
		self.bridge = CvBridge()

		# Object location initialization
		self.object_location = None

		# Subscribe to pose at all times
		self.current_pose = None
		_pose_sub = rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState, self.set_current_pose)

		return



	def set_current_pose(self, data):
		"""
		Callback for pose subscriber, sets current pose

		"""
		self.current_pose = data.pose



	def move_to_observation_pose(self):
		"""
		Move to observation position above table

		"""
		# Create the pose target
		pose_target = Pose()
		pose_target.position.x    = 0.865
		pose_target.position.y    = 0.035
		pose_target.position.z    = 0.041
		pose_target.orientation.x = 0.000
		pose_target.orientation.y = 1.000
		pose_target.orientation.z = 0.000
		pose_target.orientation.w = 0.000

		# Move left arm to pose target
		self.group.clear_pose_targets()
		self.group.set_pose_target(pose_target)
		plan = self.group.plan()
		result = False
		while not result:
			result = self.group.go() #ensure successful plan

		return



	def _on_camera(self, data):
		"""
		Image stream callback, holds all OpenCV operations

		"""
		# Convert Image to CV image (bgr8)
		try:
			img_original = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError, e:
			print("Image conversion error: ", e)

		# Convert bgr8 to hsv
		img_hsv = cv2.cvtColor(img_original, cv2.COLOR_BGR2HSV)

		# Threshold image for color red
		img_thresholded = cv2.inRange(img_hsv, np.array([self.low_h,
			                                             self.low_s,
			                                             self.low_v]),
		                                       np.array([self.high_h,
		                                       	         self.high_s,
		                                       	         self.high_v]))

		# Perform morphological opening
		img_thresholded = cv2.erode(img_thresholded, np.ones((2,2), np.uint8), iterations=1)
		img_thresholded = cv2.dilate(img_thresholded, np.ones((2,2), np.uint8), iterations=1)

		# Perform morphological closing
		img_thresholded = cv2.dilate(img_thresholded, np.ones((2,2), np.uint8), iterations=1)
		img_thresholded = cv2.erode(img_thresholded, np.ones((2,2), np.uint8), iterations=1)

		# Calculate moments of image
		moments = cv2.moments(img_thresholded)
		d_m01   = moments['m01']
		d_m10   = moments['m10']
		d_area  = moments['m00']

		# Area <= 10000 --> just noise, otherwise publish object location
		if d_area > 10000:
			size  = img_hsv.shape
			pos_x = size[0] - (d_m01 / d_area)
			pos_y = size[1] - (d_m10 / d_area)
			self.object_location = Point(pos_x, pos_y, 0.000)

		# Show original and thresholded images
		cv2.imshow('Original', img_original)
		cv2.imshow('Thresholded', img_thresholded)
		cv2.waitKey(3) #ensures images are shown

		return



	def find_block(self):
		"""
		Find block and add it to the scene

		"""
		# Get necessary values
		while not self.object_location:
			print "Waiting for object location..."
			rospy.sleep(1.0)
		object_location    = self.object_location
		image_size         = self.left_camera_size
		while not self.current_pose:
			print "Waiting for current pose values..."
			rospy.sleep(1.0)
		current_pose       = self.current_pose.position
		endeffector_height = current_pose.z - self.table_height

		# Store values for calculation
		Pp = [ object_location.x, object_location.y, object_location.z ]
		Cp = [ image_size[0] / 2, image_size[1] / 2, 0 ]
		Bp = [ current_pose.x, current_pose.y, current_pose.z ]
		Go = [ -0.0230, 0.0110, 0.1100 ]
		cc = [  0.0021, 0.0021, 0.0000 ]
		d =  [ endeffector_height, endeffector_height, 0.0000 ]

		# Calculate block's position in workspace
		# workspace = (Pp - Cp) * cc * d + Bp + Go
		pixel_difference          = map(operator.sub, Pp, Cp)
		camera_constant           = map(operator.mul, cc, d)
		pixel_2_real              = map(operator.mul, pixel_difference, camera_constant)
		pixel_2_real[2]           = endeffector_height * -1
		workspace_without_gripper = map(operator.add, pixel_2_real, Bp)
		workspace                 = map(operator.add, workspace_without_gripper, Go)

		# Add block to scene
		p = PoseStamped()
		p.header.frame_id = self.robot.get_planning_frame()
		p.pose.position.x = workspace_without_gripper[0]
		p.pose.position.y = workspace_without_gripper[1]
		p.pose.position.z = workspace_without_gripper[2] + 0.06
		co = CollisionObject()
		co.operation = CollisionObject.ADD
		co.id = "block"
		co.header = p.header
		box = SolidPrimitive()
		box.type = SolidPrimitive.BOX
		box.dimensions = list((0.1, 0.1, 0.1))
		co.primitives = [box]
		co.primitive_poses = [p.pose]
		pub_co = rospy.Publisher('collision_object', CollisionObject, latch=True)
		pub_co.publish(co)

		return



	def pick_up_block(self):
		"""
		Move Baxter's arm above the block

		"""
		rospy.sleep(2.0)
		result = False
		while not result:
			try:
				result = self.group._g.pick("block")
			except TypeError:
				print "Same grasp type error"

		return


		








def main():
	rospy.init_node('pick_place')
	pp = Pick_Place()

	pp.move_to_observation_pose()
	pp.find_block()
	pp.pick_up_block()
	#pp.move_above_block()

	print "\nDone."
	

	moveit_commander.os._exit(0)




if __name__ == '__main__':
	main()