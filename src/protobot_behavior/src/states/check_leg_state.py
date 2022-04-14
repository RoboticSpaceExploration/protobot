#!/usr/bin/env python

import rospy
import smach
import smach_ros


class CheckLeg(smach.State):
    def __init__(self, userdata):
        smach.State.__init__(self, outcomes=["success"], output_keys=["done"])
        userdata.current_leg = 0
        # Outcome: success
        # Input keys: none
        # Output keys: current leg

    def execute(self, userdata):
        rospy.loginfo("Executing state CheckLeg")
        if userdata.current_leg < 7:
            userdata.current_leg = userdata.current_leg + 1
        return "done"
        # If on leg 1-3, return early_legs
        #   increase userdata.current_leg by 1
        # If on leg 4-6, return late_legs
        #   increase userdata.current_leg by 1
        # if on leg 7, return late_legs
