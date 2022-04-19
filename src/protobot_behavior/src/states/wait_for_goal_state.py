#!/usr/bin/env python

import rospy
import smach
import smach_ros


class WaitForGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["ready"])

    def execute(self, userdata):
        rospy.loginfo("Executing state WAIT_FOR_GOAL")
        return "ready"
