#!/usr/bin/env python

import sys
import rospy
from vrep_common.msg import ( VrepInfo,
                              ProximitySensorData )
from vrep_common.srv import ( simRosEnablePublisher )


def vrep_info_callback(data):
	print data


def main():
	# Initialize ROS node
	print("\nInitializing ROS node")
	rospy.init_node("vrep_test")

	# Get joint handles and proximity senso handles from args
	args = sys.argv
	left_motor_handle = args[1]
	right_motor_handle = args[2]
	sensor_handle = args[3]

	# Subscribe to v-rep's info stream
	rospy.Subscriber('/vrep/info', VrepInfo, vrep_info_callback)

	# Request v-rep to launch proximity sensor publisher
	rospy.wait_for_service('/vrep/simRosEnablePublisher')
	try:
		enable_publisher = rospy.ServiceProxy('/vrep/simRosEnablePublisher', simRosEnablePublisher)
		resp = enable_publisher('proxData', 1, ProximitySensorData, sensor_handle, -1, "")
		print resp
	except rospy.ServiceException, e:
		print "Service call failed: %s" %e




if __name__ == "__main__":
	main()