#!/usr/bin/env python

import rospy
import smach
import smach_ros


class BeginAuto(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=["to_auto", "to_teleop"],
								   input_keys=["start_teleop", "start_sim"])
	
	def execute(self, userdata):
		rospy.loginfo("Executing state BEGIN_AUTO")
		if (userdata.start_teleop == "y"):
			return "to_teleop"
		else:
			return "to_auto"
