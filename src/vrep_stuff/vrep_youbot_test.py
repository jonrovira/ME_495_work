#!/usr/bin/env python

import sys
import tty
import termios
from select import select
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


	def getch(self, timeout=0.01):
	    """
	    Retrieves a character from stdin.

	    Returns None if no character is available within the timeout.
	    Blocks if timeout < 0.
	    """
	    # If this is being piped to, ignore non-blocking functionality
	    if not sys.stdin.isatty():
	        return sys.stdin.read(1)
	    fileno = sys.stdin.fileno()
	    old_settings = termios.tcgetattr(fileno)
	    ch = None
	    try:
	        tty.setraw(fileno)
	        rlist = [fileno]
	        if timeout >= 0:
	            [rlist, _, _] = select(rlist, [], [], timeout)
	        if fileno in rlist:
	            ch = sys.stdin.read(1)
	    except Exception as ex:
	        print "getch", ex
	        raise OSError
	    finally:
	        termios.tcsetattr(fileno, termios.TCSADRAIN, old_settings)
	    return ch


	def test(self, wheel_fl, wheel_fr, wheel_rl, wheel_rr, arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4):
		# Subscribe to v-rep's info stream
		rospy.Subscriber('/vrep/info', VrepInfo, self._vrep_info_callback)

		# Tell v-rep to subscribe to wheel speed topic
		rospy.wait_for_service('/vrep/simRosEnableSubscriber')
		enable_subscriber_req = rospy.ServiceProxy('/vrep/simRosEnableSubscriber', simRosEnableSubscriber)
		resp = enable_subscriber_req("/"+self.node_name+"/joint_state", # topicName
			                         1,                                 # queueSize
			                         0x000803,                          # streamCmd
			                         -1,                                # auxInt1
			                         -1,                                # auxInt2
			                         "")                                # auxString

		# Joint state subscription request successful
		if resp.subscriberID is not -1:

			# Create joint state publisher
			joint_state_pub = rospy.Publisher('/'+self.node_name+'/joint_state', JointSetStateData)

			# Create control loop
			while not rospy.is_shutdown() and self.simulation_running:
				
				# Get command from user
				cmd = self.getch()
				if cmd:

					# Wheel command
					if cmd in ['w','s','d','a']:

						# Initialize variables to avoid scope problems
						front_left_speed = front_right_speed = rear_left_speed = rear_right_speed = None

						# Up = move forward
						if cmd == 'w':
							front_left_speed  = 3.1415
							front_right_speed = 3.1415
							rear_left_speed   = 3.1415
							rear_right_speed  = 3.1415

						# Down = move backward
						elif cmd == 's':
							front_left_speed  = -3.1415
							front_right_speed = -3.1415
							rear_left_speed   = -3.1415
							rear_right_speed  = -3.1415	

						# Right = move right
						elif cmd == 'd':
							front_left_speed  = 3.1415
							front_right_speed = -3.1415
							rear_left_speed   = -3.1415
							rear_right_speed  = 3.1415

						# Left = move left
						elif cmd == 'a':
							front_left_speed  = -3.1415
							front_right_speed = 3.1415
							rear_left_speed   = 3.1415
							rear_right_speed  = -3.1415

						# Create ROS JointSetStateData message with appropriate data
						joint_state = JointSetStateData()
						joint_state.handles.data  = [wheel_fl, wheel_fr, wheel_rl, wheel_rr]
						joint_state.setModes.data = [2, 2, 2, 2]
						joint_state.values.data   = [front_left_speed, front_right_speed, rear_left_speed, rear_right_speed]

						# Publish control message
						joint_state_pub.publish(joint_state)
						rospy.sleep(0.1)

						# Publish 0 velocity control to stop motion
						joint_state.values.data = [0,0,0,0]
						joint_state_pub.publish(joint_state)

					# Arm command
					elif cmd in ['l']:

						joint_state = JointSetStateData()
						joint_state.handles.data  = [arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4]
						joint_state.setModes.data = [0, 0, 0, 0]
						joint_state.values.data   = [0, 0, 0, 0,]

						joint_state_pub.publish(joint_state)
						rospy.sleep(0.5)



		# Request wasn't successful
		else: print "Unsuccessful wheel speed topic subscription request"



def main():
	# Get Youbot handles
	wheel_fl    = int(sys.argv[1])
	wheel_fr    = int(sys.argv[2])
	wheel_rl    = int(sys.argv[3])
	wheel_rr    = int(sys.argv[4])
	arm_joint_1 = int(sys.argv[5])
	arm_joint_2 = int(sys.argv[6])
	arm_joint_3 = int(sys.argv[7])
	arm_joint_4 = int(sys.argv[8])

	# Launch Vrep Baxter Test
	vbt = Vrep_Youbot_Test()
	vbt.test(wheel_fl, wheel_fr, wheel_rl, wheel_rr, arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4)



if __name__ == "__main__":
	main()