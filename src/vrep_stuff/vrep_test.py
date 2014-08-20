#!/usr/bin/env python

import sys
import rospy
from vrep_common.msg import ( VrepInfo,
                              ProximitySensorData,
                              JointSetStateData )
from vrep_common.srv import ( simRosEnablePublisher,
                              simRosEnableSubscriber )



class Vrep_Test(object):

	def __init__(self):
		# Global initialization
		self.node_name          = "vrep_test"
		self.simulation_running = True
		self.sensor_trigger     = False
		self.simulation_time    = 0.0

		# Initialize ROS node
		rospy.init_node(self.node_name)
		print "\n" + self.node_name + " node started"


	def _vrep_info_callback(self, data):
		# /vrep/info subscription callback
		self.simulation_time = data.simulationTime.data
		self.simulation_running = int(data.simulatorState.data) != 0


	def _vrep_proximity_sensor_data_callback(self, data):
		# /vrep/proxData subscription callback
		self.sensor_trigger = True


	def test(self, l_motor_handle, r_motor_handle, sensor_handle):
		# Subscribe to v-rep's info stream
		rospy.Subscriber('/vrep/info', VrepInfo, self._vrep_info_callback)

		# Request v-rep to launch proximity sensor publisher
		rospy.wait_for_service('/vrep/simRosEnablePublisher')
		enable_publisher_req = rospy.ServiceProxy('/vrep/simRosEnablePublisher', simRosEnablePublisher)
		resp = enable_publisher_req('proxData',         # topicName
			                        1,                  # queueSize
			                        0x001010,           # streamCmd
			                        int(sensor_handle), # auxInt1
			                        -1,                 # auxInt2
			                        "")                 # auxString

		# Proximity sensor topic publish request sucessful
		if len(resp.effectiveTopicName) is not 0:

			# Subscribe to the published topic
			topic_name = "/vrep/" + resp.effectiveTopicName
			rospy.Subscriber(topic_name, ProximitySensorData, self._vrep_proximity_sensor_data_callback)

			# Tell v-rep to subscribe to motor speed topic
			rospy.wait_for_service('/vrep/simRosEnableSubscriber')
			enable_subscriber_req = rospy.ServiceProxy('/vrep/simRosEnableSubscriber', simRosEnableSubscriber)
			resp = enable_subscriber_req("/"+self.node_name+"/wheels", # topicName
				                         1,                            # queueSize
				                         0x000803,                     # streamCmd
				                         -1,                           # auxInt1
				                         -1,                           # auxInt2
				                         "")                           # auxString

			# Motor speed topic subscription request successful
			if resp.subscriberID is not -1:
				
				# Create motor speed publisher
				wheel_pub = rospy.Publisher('/'+self.node_name+'/wheels', JointSetStateData)

				# Create control loop
				drive_back_start_time = -99.0
				while not rospy.is_shutdown() and self.simulation_running:

					motor_speeds = JointSetStateData()

					if self.simulation_time-drive_back_start_time < 3.0:
						# Drive backwards, slightly turning
						desired_left_motor_speed  = -3.1415*0.5
						desired_right_motor_speed = -3.1415*0.25
						print "Driving backwards"

					else:
						# Drive forwards
						desired_left_motor_speed  = 3.1415
						desired_right_motor_speed = 3.1415
						if self.sensor_trigger:
							drive_back_start_time = self.simulation_time
						self.sensor_trigger = False
						print "Driving forwards"

					# Publish motor speeds
					motor_speeds.handles.data.append(l_motor_handle)
					motor_speeds.handles.data.append(r_motor_handle)
					motor_speeds.setModes.data = [2,2]
					motor_speeds.values.data.append(desired_left_motor_speed)
					motor_speeds.values.data.append(desired_right_motor_speed)
					wheel_pub.publish(motor_speeds)
					rospy.sleep(3.0)

			# Request wasn't successful
			else: print "Unsuccessful motor speed topic subscription request"

		# Request wasn't successful
		else: print "Unsuccessful proximity sensor publisher request"



def main():
	# Get joint handles and proximity senso handles from args
	l_motor_handle = int(sys.argv[1])
	r_motor_handle = int(sys.argv[2])
	sensor_handle  = int(sys.argv[3])

	# Launch Vrep Test
	vt = Vrep_Test()
	vt.test(l_motor_handle, r_motor_handle, sensor_handle)






if __name__ == "__main__":
	main()