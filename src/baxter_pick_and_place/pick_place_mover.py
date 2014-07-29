#!/usr/bin/env python

"""
Description:

Node that controls Baxter's arm movements using Moveit pose planning. The node
receives information from other topics published from different nodes.

Axes used:
(0, 0, 0): Center of Baxter's torso
+x: Protruding forward from Baxter
+y: Protruding to Baxter's left
+z: Protruding up from Baxter

Help from: docs.ros.org/hydro/api/pr2_moveit_tutorials/html
          /planning/scripts/doc/move_group_python_interface_tutorial.html
"""


import sys
import copy

import rospy
import moveit_commander
from moveit_commander import conversions

from std_msgs.msg import Int32
from geometry_msgs.msg import Pose, PoseStamped

import baxter_interface
from baxter_core_msgs.msg import EndpointState
import moveit_msgs.msg

from baxter_core_msgs.srv import ( SolvePositionIK,
								   SolvePositionIKRequest )
from jon_baxter.srv import Coordinate

class Pick_Place_Mover(object):

	def __init__(self):
		"""
		Initialize Pick_Place_Mover class
		"""
		# Initialize Baxter's arm
		self.left_arm = baxter_interface.Limb("left")
		self.left_arm.set_joint_position_speed(0.5)

		# Initialize IK service
		self.ik_service = rospy.ServiceProxy(
			"ExternalTools/left/PositionKinematicsNode/IKService",
			SolvePositionIK)	

		# Wait for RViz to start
		rospy.sleep(5)

		# Subscribe to Baxter's end-effector pose
		self._pose_sub = rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, self.set_pose)

		# Subscribe to pose queries from Pick Place Controller node
		self._pose_query_sub = rospy.Subscriber("/pick_place_controller/pose_query", Pose, self.set_pose_query)
		self.pose_query = None
		return


	def set_available(self):
		"""
		Sets Mover node to available
		"""
		# Try setting Mover node to available
		try:
			coordinate = rospy.ServiceProxy('coordinate', Coordinate)
			resp = coordinate(
				              'pick_place_mover', #node
			                  0,                  #status_request
			                  1,                  #status_set
			                  1)                  #status_setting

			# Return success (1 or 0)
			return resp.successful

		# Service exception
		except rospy.ServiceException, e:
			print "==[MOVER]== Service call failed: %s" %e
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
				              'pick_place_mover', #node
			                  0,       #status_request
			                  1,       #status_set
			                  0)       #status_setting

			# Return success (1 or 0)
			return resp.successful

		# Service exception
		except rospy.ServiceException, e:
			print "==[MOVER]== Service call failed: %s" %e
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
			print "==[MOVER]== Service call failed: %s" %e
			# Returns unsuccessful
			return 0


	def set_pose(self, data):
		"""
		Sets/updates pose variable from pose subscriber
		"""
		self.current_pose = data.pose
		return


	def set_pose_query(self, data):
		"""
		Sets/updates pose query variable from pose query subscriber
		"""
		self.pose_query = data
		return


	def request_pose(self, pose):
		# Set stamped pose
		pose_stamped = PoseStamped()
		pose_stamped.pose = pose
		pose_stamped.header.frame_id = "base"
		pose_stamped.header.stamp = rospy.Time.now()

		# Create IK request 
		ik_request = SolvePositionIKRequest()
		ik_request.pose_stamp.append(pose_stamped)

		# Request service
		try:
			rospy.wait_for_service("ExternalTools/left/PositionKinematicsNode/IKService", 5.0)
			ik_response = self.ik_service(ik_request)
		except (rospy.ServiceException, rospy.ROSException), error_message:
			rospy.logerr("Service request failed: %r" %(error_message))
			sys.exit("==[MOVER]== ERROR - move_to_observe - Failed to append pose")
		if ik_response.isValid[0]:
			limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
			self.left_arm.move_to_joint_positions(limb_joints)
		else:
			sys.exit("==[MOVER]== ERROR - move_above_position - No valid joint configuration found")

		return


	def move_to_observe(self, setbusy=True, setavailable=True):
		"""
		Moves move group end-effector to a position at which it
		can observe the entire space
		"""
		# Set node to busy
		if setbusy:
			self.set_busy()

		# Create the necessary pose
		pose = Pose()
		pose.orientation.x = 0.000
		pose.orientation.y = 1.000
		pose.orientation.z = 0.000
		pose.orientation.w = 0.000
		pose.position.x = 0.765
		pose.position.y = 0.035
		pose.position.z = 0.035

		# Request service
		self.request_pose(pose)

		# Set node to available
		if setavailable:
			self.set_available()
		return


	def move_above_position(self, setbusy=True, setavailable=True):
		"""
		Moves move group end-effector to a position at which it
		is directly above an object whose pose is queried
		"""
		# Set node to busy
		if setbusy:
			self.set_busy()

		# Get pose query
		pose_query = self.pose_query

		# Start with current pose
		current_pose = self.current_pose

		# Set x and y coordinates for pose
		pose = Pose()
		pose.orientation.x = 0.0
		pose.orientation.y = 1.0
		pose.orientation.z = 0.0
		pose.orientation.w = 0.0
		pose.position.x = pose_query.position.x
		pose.position.y = pose_query.position.y
		pose.position.z = current_pose.position.z

		# Request service
		self.request_pose(pose)

		# Set node to available
		if setavailable:
			self.set_available()
		return


	def move_down_position(self, setbusy=True, setavailable=True):
		"""
		Moves move group end-effector to a position at which it
		is down around the object whose pose is queried
		"""
		# Set node to busy
		if setbusy:
			self.set_busy()

		# Initialize pose query
		pose_query = self.pose_query

		# Start with current pose
		current_pose = self.current_pose

		# Set pose
		pose = Pose()
		pose.orientation.x = 0.0
		pose.orientation.y = 1.0
		pose.orientation.z = 0.0
		pose.orientation.w = 0.0
		pose.position.x = pose_query.position.x
		pose.position.y = pose_query.position.y
		pose.position.z = pose_query.position.z
		
		# Request service
		self.request_pose(pose)

		# Set node to available
		if setavailable:
			self.set_available()
		return


	def move_to_drop_position(self, setbusy=True, setavailable=True):
		"""
		Moves end-effector to a position at which it can drop the Rubik's
		cube
		"""
		# Set node to busy
		if setbusy:
			self.set_busy()

		# Set pose
		pose = Pose()
		pose.orientation.x = 0.0
		pose.orientation.y = 1.0
		pose.orientation.z = 0.0
		pose.orientation.w = 0.0
		pose.position.x = 0.62
		pose.position.y = -0.18
		pose.position.z = -0.20

		# Request service
		self.request_pose(pose)

		# Set node to available
		if setavailable:
			self.set_available()
		return




def main():
	print("==[MOVER]== Initializing node")
	rospy.init_node('pick_place_mover')
	rospy.wait_for_service('coordinate')
	wait_rate = rospy.Rate(1)

	print("==[MOVER]== Starting Mover...")
	ppm = Pick_Place_Mover()

	print("==[MOVER]== Moving to observation pose")
	ppm.move_to_observe()
	while not ppm.check_availability('pick_place_controller'):
		wait_rate.sleep()

	print("==[MOVER]== Moving above Rubik's cube")
	ppm.move_above_position()
	while not ppm.check_availability('pick_place_controller'):
		wait_rate.sleep()

	print("==[MOVER]== Moving down to Rubik's cube")
	ppm.move_down_position()
	while not ppm.check_availability('pick_place_controller'):
		wait_rate.sleep()

	print("==[MOVER]== Moving to drop pose")
	ppm.move_to_observe(setavailable=False)
	ppm.move_to_drop_position(setbusy=False)
	while not ppm.check_availability('pick_place_controller'):
		wait_rate.sleep()
	ppm.move_to_observe()

	while not rospy.is_shutdown():
		rospy.spin()
	print("\n==[MOVER]== Mover done.")


if __name__ == '__main__':
	main()

