#!/usr/bin/env python

import rospy
import smach
import smach_ros
import roslaunch
#import subprocess could use for opening in new terminal window - run start_teleop.py in new terminal?

class Teleop(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['complete'],
								   input_keys=['start_teleop'],
								   output_keys=['start_teleop'])
		
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)
		launch = roslaunch.parent.ROSLaunchParent (uuid, ['/home/roselab/protobot/src/protobot_teleop/launch/protobot_teleop.launch'])
		launch.start()
		rospy.loginfo('Launched: Teleop')
	
	def execute(self, userdata):
		rospy.loginfo('Executing state TELEOP')
		userdata.start_teleop = input("continue?\n")
		
		return 'complete'
