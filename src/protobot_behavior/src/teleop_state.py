#!/usr/bin/env python

import rospy
import smach
import smach_ros
import subprocess

class Teleop(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['complete'],
								   input_keys=['start_teleop'],
								   output_keys=['start_teleop'])
	
	def execute(self, userdata):
		rospy.loginfo('Executing state TELEOP')
		
		# Start teleop node
		subprocess.run('gnome-terminal -- bash -c "roslaunch protobot_teleop protobot_teleop.launch; exec bash"', shell=True)
		rospy.loginfo('Launched: Teleop')
		
		# Wait until user wants to kill teleop node
		kill = input("KILL TELEOP? (y)\n")
		while (kill != 'y'):
			kill = input("KILL TELEOP? (y)\n")
		# Kill teleop node in new window
		subprocess.Popen('rosnode kill teleop; exec bash', shell=True)
		rospy.loginfo('Killing: Teleop node')
		
		userdata.start_teleop = ''
		rospy.Rate(1)
		# Wait 1 sec
		rospy.sleep(1)
		return 'complete'
