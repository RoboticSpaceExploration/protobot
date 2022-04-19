#!/usr/bin/env python

import rospy
import smach
import smach_ros
import subprocess

class Simulation(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=["complete"])
	
	def execute(self, userdata):
		subprocess.Popen("roslaunch protobot_bringup protobot_simulation.launch", shell=True)
		#subprocess.run('gnome-terminal -- bash -c "roslaunch protobot_bringup protobot_simulation.launch; exec bash"', shell=True)
		rospy.loginfo("Launched: Simulation")
		input("enter when done")
		return "complete"
