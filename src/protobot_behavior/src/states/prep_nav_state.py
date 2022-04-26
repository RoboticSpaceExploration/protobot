#!/usr/bin/env python

import rospy
import smach
import smach_ros


class PrepNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["to_nav", "finish"], input_keys=["current_leg"])

    def execute(self, userdata):
        rospy.loginfo("Executing state PREP_NAV")
        return "to_nav"
        return "finish"
