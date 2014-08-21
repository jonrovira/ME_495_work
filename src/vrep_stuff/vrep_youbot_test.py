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


class Vrep_Youbot_Test(object):

	def __init__(self):
		# Global initialization
		self.node_name          = "vrep_youbot_test"
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


	def test(self, wheel_fl, wheel_fr, wheel_rl, wheel_rr, arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4):
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

			def set_wheels(wheels, directions):
				joint_state = JointSetStateData()
				joint_state.handles.data  = wheels
				joint_state.setModes.data = len(wheels) * [2]
				joint_state.values.data   = [directions[x] * 3.1415 for x in range(len(wheels))]
				self.joint_state_pub.publish(joint_state)
				rospy.sleep(0.1)
				joint_state.values.data = len(wheels) * [0]
				self.joint_state_pub.publish(joint_state)

			def set_joints(joints, directions):
				current_angles = []
				current_angles = [self.joint_state_req(joints[x]).state.position[0] for x in range(len(joints))]
				joint_state = JointSetStateData()
				joint_state.handles.data  = joints
				joint_state.setModes.data = len(joints) * [0]
				joint_state.values.data   = [(current_angles[x] + (directions[x] * 0.06)) for x in range(len(directions))]
				self.joint_state_pub.publish(joint_state)

			bindings = {
				'w': (set_wheels, [wheel_fl, wheel_fr, wheel_rl, wheel_rr], [ 1,  1,  1,  1]),
				's': (set_wheels, [wheel_fl, wheel_fr, wheel_rl, wheel_rr], [-1, -1, -1, -1]),
				'd': (set_wheels, [wheel_fl, wheel_fr, wheel_rl, wheel_rr], [ 1, -1, -1,  1]),
				'a': (set_wheels, [wheel_fl, wheel_fr, wheel_rl, wheel_rr], [-1,  1,  1, -1]),
				'n': (set_joints, [arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4], [ 1,  0,  0,  0]),
				'm': (set_joints, [arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4], [-1,  0,  0,  0]),
				'j': (set_joints, [arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4], [ 0,  1,  0,  0]),
				'k': (set_joints, [arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4], [ 0, -1,  0,  0]),
				'i': (set_joints, [arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4], [ 0,  0,  1,  0]),
				'o': (set_joints, [arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4], [ 0,  0, -1,  0]),
				'9': (set_joints, [arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4], [ 0,  0,  0,  1]),
				'0': (set_joints, [arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4], [ 0,  0,  0, -1]),
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