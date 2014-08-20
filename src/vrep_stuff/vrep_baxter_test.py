#!/usr/bin/env python

import sys
import rospy
from vrep_common.msg import ( VrepInfo )


class Vrep_Baxter_Test(object):

	def __init__(self):
		# Global initialization
		self.node_name          = "vrep_baxter_test"
		self.simulation_running = True

		# Initiailize ROS node
		rospy.init_node(self.node_name)
		print "\n" + self.node_name + " node started"


	def _vrep_info_callback(self, data):
		# /vrep/info subscription callback
		self.simulation_running = int(data.simulatorState.data) != 0


	def test(self, l_arm, r_arm):
		# Subscribe to v-rep's info stream
		rospy.Subscriber('/vrep/info', VrepInfo, self._vrep_info_callback)

		while self.simulation_running:
			rospy.sleep(1.0)
			print "HI"



def main():
	# Get arm handles
	l_arm = int(sys.argv[1])
	r_arm = int(sys.argv[2])

	# Launch Vrep Baxter Test
	vbt = Vrep_Baxter_Test()
	vbt.test(l_arm, r_arm)



if __name__ == "__main__":
	main()