#!/usr/bin/env python

import rospy
import smach
import smach_ros

import teleop_state
import auto_state

def main():
	rospy.init_node('smach_state_machine')
	
	teleop_sm = smach.StateMachine(outcomes=['SUCCEEDED', 'FAILED'])
	
	teleop_sm.userdata.start_teleop = input("BEGIN TELEOP?\n")
	
	with teleop_sm:
		smach.StateMachine.add('BEGIN_AUTO', auto_state.BeginAuto(),
							   transitions={'complete':'TELEOP',
							   				'running':'FAILED'})
		smach.StateMachine.add('TELEOP', teleop_state.Teleop(),
							   transitions={'complete':'BEGIN_AUTO'})
	
	outcome = teleop_sm.execute()
	
if __name__ == '__main__':
	main()
	rospy.spin()
