#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import baxter_interface
from sensor_msgs.msg import JointState
from baxter_core_msgs.msg import EndEffectorState

class Joint_Gripper_State_Merger():

    def __init__(self):
        """
        Initialize Joint_Gripper_State_Merger class

        """
        # Instantiations
        self.left_gripper = None
        self.right_gripper = None
        self.joint_states = None

        # Names initializations
        self.l_finger_joints = ["left_gripper_l", "left_gripper_r"]
        self.r_finger_joints = ["right_gripper_l", "right_gripper_r"]

        # Constants
        upper = 0.0095
        self.lower = -0.0125
        self.diff = (upper - self.lower)/100

        # Merged state publisher
        self.state_pub = rospy.Publisher("/merged/joint_states", JointState)

        # Gripper state subscribers
        self.left_gripper_state_sub = rospy.Subscriber("/robot/end_effector/left_gripper/state", EndEffectorState, self.updateLeftGripperState)
        self.right_gripper_state_sub = rospy.Subscriber("/robot/end_effector/right_gripper/state", EndEffectorState, self.updateRightGripperState)
        #rospy.sleep(3.0) # Get at least 1 value stored

        # Robot joint state subscribers
        self.joint_states_sub = rospy.Subscriber("/robot/joint_states", JointState, self.updateJointState)

        rospy.loginfo("Joint gripper state merger initialized!")

        
    def updateJointState(self, msg):
        """
        Publish merged joint states

        """
        self.joint_states = msg
        self.mergeStates()
        if len(self.joint_states.name) == 21:
            self.state_pub.publish(self.joint_states)
        

    def updateLeftGripperState(self, msg):
        """
        Store left gripper states after receiving each EndEffectorState message from left gripper

        """
        self.left_gripper = msg
        

    def updateRightGripperState(self, msg):
        """
        Store right gripper states after receiving each EndEffectorState message from right gripper

        """
        self.right_gripper = msg
        

    def mergeStates(self):
        """
        Merge joint states with gripper states, store result

        """
        try:
            for joint in self.l_finger_joints:
                self.joint_states.name.append(joint+"_finger_joint")
                self.joint_states.position  = self.joint_states.position + (self.lower + self.diff * self.left_gripper.position,)
                self.joint_states.velocity =  self.joint_states.velocity +  (0.0,)
                self.joint_states.effort = self.joint_states.effort +(0.0,)
            for joint in self.r_finger_joints:
                self.joint_states.name.append(joint+"_finger_joint")
                self.joint_states.position  = self.joint_states.position + (self.lower + self.diff * self.right_gripper.position,)
                self.joint_states.velocity =  self.joint_states.velocity +  (0.0,)
                self.joint_states.effort = self.joint_states.effort +(0.0,)
        except:
            rospy.logwarn("Could not update left finger position")



def main():
    rospy.init_node("joint_gripper_publisher")
    print "Starting..."

    jgsm = Joint_Gripper_State_Merger()
    
    while not rospy.is_shutdown():
        rospy.spin()
    print "Done"

    
if __name__ == '__main__':
    main()