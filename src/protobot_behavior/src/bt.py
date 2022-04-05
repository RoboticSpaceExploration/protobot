#!/usr/bin/env python

import rospy
import smach
import smach_ros

import teleop_state
import auto_state
import simulation_state as sim_state

def main():
	rospy.init_node('smach_state_machine')
	
	teleop_sm = smach.StateMachine(outcomes=['SUCCEEDED', 'FAILED'])
	
	teleop_sm.userdata.start_teleop = input("BEGIN TELEOP? (y)\n")
	
	with teleop_sm:
		smach.StateMachine.add('BEGIN_AUTO', auto_state.BeginAuto(),
							   transitions={'complete':'TELEOP',
							   				'running':'SIMULATION'})
		smach.StateMachine.add('TELEOP', teleop_state.Teleop(),
							   transitions={'complete':'BEGIN_AUTO'})
		smach.StateMachine.add('SIMULATION', sim_state.Simulation(),
							   transitions={'complete':'SUCCEEDED'})
	
	outcome = teleop_sm.execute()
	
if __name__ == '__main__':
	main()
	# SHOULD IT TERMINATE ITSELF AFTER COMPLETION
	rospy.spin()
