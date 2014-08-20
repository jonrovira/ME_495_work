#!/usr/bin/env python

import sys
import rospy
from vrep_common.msg import ( VrepInfo,
                              JointSetStateData )
from vrep_common.srv import ( simRosEnableSubscriber )


class Vrep_Youbot_Test(object):

	def __init__(self):
		# Global initialization
		self.node_name          = "vrep_youbot_test"
		self.simulation_running = True

		# Initiailize ROS node
		rospy.init_node(self.node_name)
		print "\n" + self.node_name + " node started"


	def _vrep_info_callback(self, data):
		# /vrep/info subscription callback
		self.simulation_running = int(data.simulatorState.data) != 0


	def test(self, wheel_fl, wheel_fr, wheel_rl, wheel_rr):
		# Subscribe to v-rep's info stream
		rospy.Subscriber('/vrep/info', VrepInfo, self._vrep_info_callback)

		# Tell v-rep to subscribe to wheel speed topic
		rospy.wait_for_service('/vrep/simRosEnableSubscriber')
		enable_subscriber_req = rospy.ServiceProxy('/vrep/simRosEnableSubscriber', simRosEnableSubscriber)
		resp = enable_subscriber_req("/"+self.node_name+"wheels", # topicName
			                         1,                           # queueSize
			                         0x000803,                    # streamCmd
			                         -1,                          # auxInt1
			                         -1,                          # auxInt2
			                         "")                          # auxString

		# Wheel speed topic subscription request successful
		if resp.subscriberID is not -1:

			# Create wheel speed publisher
			wheel_pub = rospy.Publisher('/'+self.node_name+'/wheels', JointSetStateData)

			# Create control loop
			while not rospy.is_shutdown() and self.simulation_running:
				print "Hi"
				
				wheel_speeds = JointSetStateData()

				front_left_speed  = 3.1415
				front_right_speed = 3.1415
				rear_left_speed   = 3.1415
				rear_right_speed  = 3.1415

				wheel_speeds.handles.data.append(wheel_fl)
				wheel_speeds.handles.data.append(wheel_fr)
				wheel_speeds.handles.data.append(wheel_rl)
				wheel_speeds.handles.data.append(wheel_rr)
				wheel_speeds.setModes.data = [2,2,2,2]
				wheel_speeds.values.data.append(front_left_speed)
				wheel_speeds.values.data.append(front_right_speed)
				wheel_speeds.values.data.append(rear_left_speed)
				wheel_speeds.values.data.append(rear_right_speed)

				wheel_pub.publish(wheel_speeds)
				rospy.sleep(3.0)

		# Request wasn't successful
		else: print "Unsuccessful wheel speed topic subscription request"



def main():
	# Get arm handles
	wheel_fl = int(sys.argv[1])
	wheel_fr = int(sys.argv[2])
	wheel_rl = int(sys.argv[3])
	wheel_rr = int(sys.argv[4])

	# Launch Vrep Baxter Test
	vbt = Vrep_Youbot_Test()
	vbt.test(wheel_fl, wheel_fr, wheel_rl, wheel_rr)



if __name__ == "__main__":
	main()