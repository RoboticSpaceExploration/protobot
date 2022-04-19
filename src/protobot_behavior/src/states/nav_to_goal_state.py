#!/usr/bin/env python

import rospy
import smach
import smach_ros


class NavToGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["reached_goal", "start_search", "abort_path"])

    def execute(self, userdata):
        rospy.loginfo("Executing state NAME")
        return "reached_goal"
        return "start_search"
        return "abort_path"