#!/usr/bin/env python

import rospy
import smach
import smach_ros


class NavToGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["reached_goal", "aborted_goal", "start_search"],
                                   input_keys=["coord"])

    def execute(self, userdata):
        rospy.loginfo("Executing state NAV_TO_GOAL")
        userdata.coord
        return "reached_goal"
        return "aborted_goal"
        return "start_search"