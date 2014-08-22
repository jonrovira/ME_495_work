#!/usr/bin/env python

import sys
import tty
import termios
from select import select
import rospy
from vrep_common.msg import ( VrepInfo,
                              JointSetStateData )
from vrep_common.srv import ( simRosEnableSubscriber,
                              simRosGetJointState )


class Vrep_Baxter_Test(object):

	def __init__(self):
		# Global initialization
		self.node_name          = "vrep_baxter_test"
		self.simulation_running = True

		# Initiailize ROS node
		rospy.init_node(self.node_name)
		rospy.wait_for_service('/vrep/simRosEnableSubscriber')
		rospy.wait_for_service('/vrep/simRosGetJointState')
		self.joint_state_req = rospy.ServiceProxy('/vrep/simRosGetJointState', simRosGetJointState)
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


	def test(self, l_joint_1, l_joint_2, l_joint_3, l_joint_4, l_joint_5, l_joint_6, l_joint_7, r_joint_1, r_joint_2, r_joint_3, r_joint_4, r_joint_5, r_joint_6, r_joint_7):
		# Subscribe to v-rep's info stream
		rospy.Subscriber('/vrep/info', VrepInfo, self._vrep_info_callback)

		# Tell v-rep to subscribe to wheel speed topic
		enable_subscriber_req = rospy.ServiceProxy('/vrep/simRosEnableSubscriber', simRosEnableSubscriber)
		resp = enable_subscriber_req("/"+self.node_name+"/joint_state", # topicName
			                         1,                                 # queueSize
			                         0x000803,                          # streamCmd
			                         -1,                                # auxInt1
			                         -1,                                # auxInt2
			                         "")                                # auxString

		if resp.subscriberID is not -1:
			self.joint_state_pub = rospy.Publisher('/'+self.node_name+'/joint_state', JointSetStateData)

			def set_joints(joint, direction):
				current_angle = self.joint_state_req(joint).state.position[0]
				joint_state = JointSetStateData()
				joint_state.handles.data  = [joint]
				joint_state.setModes.data = [1]
				joint_state.values.data   = [current_angle + (direction * 0.1)]
				self.joint_state_pub.publish(joint_state)

			bindings = {
				'1': (set_joints, l_joint_1,  1),
				'!': (set_joints, l_joint_1, -1),
				'2': (set_joints, l_joint_2,  1),
				'@': (set_joints, l_joint_2, -1),
				'3': (set_joints, l_joint_3,  1),
				'#': (set_joints, l_joint_3, -1),
				'4': (set_joints, l_joint_4,  1),
				'$': (set_joints, l_joint_4, -1),
				'5': (set_joints, l_joint_5,  1),
				'%': (set_joints, l_joint_5, -1),
				'6': (set_joints, l_joint_6,  1),
				'^': (set_joints, l_joint_6, -1),
				'7': (set_joints, l_joint_7,  1),
				'&': (set_joints, l_joint_7, -1),

				'a': (set_joints, r_joint_1,  1),
				'A': (set_joints, r_joint_1, -1),
				's': (set_joints, r_joint_2,  1),
				'S': (set_joints, r_joint_2, -1),
				'd': (set_joints, r_joint_3,  1),
				'D': (set_joints, r_joint_3, -1),
				'f': (set_joints, r_joint_4,  1),
				'F': (set_joints, r_joint_4, -1),
				'g': (set_joints, r_joint_5,  1),
				'G': (set_joints, r_joint_5, -1),
				'h': (set_joints, r_joint_6,  1),
				'H': (set_joints, r_joint_6, -1),
				'j': (set_joints, r_joint_7,  1),
				'J': (set_joints, r_joint_7, -1),
			}

			while not rospy.is_shutdown() and self.simulation_running:
				c = self.getch()
				if c:
					if c in bindings:
						cmd = bindings[c]
						cmd[0](cmd[1], cmd[2])
					else:
						print("key bindings: ")
						print("  ?: Help")
						print("Possible controls: ")
						for key in bindings:
							print key


		else: print "Unsuccessful wheel speed topic subscription request"



def main():
	# Get Youbot handles
	l_joint_1 = int(sys.argv[1])
	l_joint_2 = int(sys.argv[2])
	l_joint_3 = int(sys.argv[3])
	l_joint_4 = int(sys.argv[4])
	l_joint_5 = int(sys.argv[5])
	l_joint_6 = int(sys.argv[6])
	l_joint_7 = int(sys.argv[7])
	r_joint_1 = int(sys.argv[8])
	r_joint_2 = int(sys.argv[9])
	r_joint_3 = int(sys.argv[10])
	r_joint_4 = int(sys.argv[11])
	r_joint_5 = int(sys.argv[12])
	r_joint_6 = int(sys.argv[13])
	r_joint_7 = int(sys.argv[14])

	# Launch Vrep Baxter Test
	vbt = Vrep_Baxter_Test()
	vbt.test(l_joint_1, l_joint_2, l_joint_3, l_joint_4, l_joint_5, l_joint_6, l_joint_7,
		     r_joint_1, r_joint_2, r_joint_3, r_joint_4, r_joint_5, r_joint_6, r_joint_7)



if __name__ == "__main__":
	main()