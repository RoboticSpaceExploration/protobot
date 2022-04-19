#!/usr/bin/env python

import rospy
import smach
import smach_ros


class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["goal_found"])

    def execute(self, userdata):
        rospy.loginfo("Executing state SEARCH")
        return "goal_found"
