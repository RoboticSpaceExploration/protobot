#!/usr/bin/env python

import rospy
import smach
import smach_ros


class Name(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[""], input_keys=[""])

    def execute(self, userdata):
        rospy.loginfo("Executing state NAME")
        return
