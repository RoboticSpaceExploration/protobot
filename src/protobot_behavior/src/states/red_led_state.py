#!/usr/bin/env python

import rospy
import smach
import smach_ros


class FlashRed(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done_flash_red"])

    def execute(self, userdata):
        rospy.loginfo("Executing state FlashRed")
        # Flash red led

        # run LED array client node
        # rosrun protobot_hardware protobot_hardware_client_node 2
        return "done_flash_red"
