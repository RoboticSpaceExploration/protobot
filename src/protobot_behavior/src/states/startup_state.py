#!/usr/bin/env python

import rospy
import smach
import smach_ros


class BeginMission(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=["to_auto", "to_teleop"],
                                   input_keys=["start_teleop"],
                                   output_keys=["current_leg"])
	
	def execute(self, userdata):
		rospy.loginfo("Executing state BEGIN_MISSION")
        userdata.current_leg = 0

		if (userdata.start_teleop == "y"):
			return "to_teleop"
		else:
			return "to_auto"
