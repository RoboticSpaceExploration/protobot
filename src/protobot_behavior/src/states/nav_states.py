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

class NavToGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["reached_goal", "aborted_goal", "start_search"],
                                   input_keys=["coord"],
                                   output_keys=["current_leg"])

    def execute(self, userdata):
        rospy.loginfo("Executing state NAV_TO_GOAL")
        # userdata.coord....
        # if reached goal, increment
        # userdata.current_leg += 1
        return "reached_goal"
        return "aborted_goal"
        return "start_search"