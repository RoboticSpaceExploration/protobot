#!/usr/bin/env python

import rospy
import smach
import smach_ros
import subprocess

class BeginAuto(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['running', 'complete'],
								   input_keys=['start_teleop'],
								   output_keys=['start_teleop'])
	
	def execute(self, userdata):
		rospy.loginfo('Executing state BEGIN_AUTO')
		if (userdata.start_teleop == 'y'):
			return 'complete'
		else:
			return 'running'
