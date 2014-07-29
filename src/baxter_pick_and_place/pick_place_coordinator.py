#!/usr/bin/env python


"""
Description:

"""

import rospy

from jon_baxter.srv import Coordinate, CoordinateResponse



class Pick_Place_Coordinator(object):

	def __init__(self):
		self.service = rospy.Service('coordinate', Coordinate, self.handle_coordination)
		self.node_availabilities = {'pick_place_vision':      0,
		                            'pick_place_mover':       0,
		                            'pick_place_controller':  0,
		                            'pick_place_coordinator': 0}


	def handle_coordination(self, req):
		"""
		Handles a Coordination service request
		"""
		# Check for valid node
		if req.node in self.node_availabilities:

			# Status request
			if req.status_request:
				rospy.sleep(1.0) #when other node setting value simultaneously
				available = self.node_availabilities[req.node]
				successful = 1

			# Status set
			elif req.status_set:
				self.node_availabilities[req.node] = req.status_setting
				available = -1 # n/a
				successful = 1

		# Invalid node
		else:
			available = -1 #n/a
			successful = 0

		# Return Coordinate Response
		return CoordinateResponse(available, successful)




def main():
	print("==[PICK PLACE COORDINATOR]== Initializing node")
	rospy.init_node('pick_place_coordinator')

	print("==[PICK PLACE COORDINATOR]== Starting Pick Place Coordinator...")
	ppc = Pick_Place_Coordinator()

	while not rospy.is_shutdown():
		rospy.spin()
	print("\n==[PICK PLACE COORDINATOR]== Pick Place Coordinator done.")


if __name__ == "__main__":
	main()

