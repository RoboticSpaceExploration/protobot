#!/usr/bin/env python

import rospy
import smach
import smach_ros
import subprocess

class FlashRed(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["complete"])

    def execute(self, userdata):
        rospy.loginfo("Executing state FLASH_RED")
        # Flash red led
        # run LED array client node
        subprocess.Popen('rosrun protobot_hardware protobot_hardware_client_node 2', shell=True)
        return 'complete'

class FlashGreen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["complete"])

    def execute(self, userdata):
        rospy.loginfo("Executing state FLASH_RED")
        # Flash green led
        # run LED array client node
        subprocess.Popen('rosrun protobot_hardware protobot_hardware_client_node 2', shell=True)
        return 'complete'
