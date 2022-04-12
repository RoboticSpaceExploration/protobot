#!/usr/bin/env python3.8

import rospy
import smach
import smach_ros

import teleop_state
import auto_state
import simulation_state as sim_state

def main():
	rospy.init_node('smach_state_machine')
	
	bt = smach.StateMachine(outcomes=['BEGIN', 'END'])
	
	with bt:
		
		setup_sm = smach.StateMachine(outcomes=['SUCCEEDED', 'FAILED'])
		
		with setup_sm:
			# Prompt if teleop or simulation should start
			# Comment out if statement to remove simulation prompt
			setup_sm.userdata.start_teleop = input("BEGIN TELEOP? (y)\n")
			if (setup_sm.userdata.start_teleop != 'y'):
				setup_sm.userdata.start_sim = input("BEGIN SIMULATION? (y)\n")
				
			smach.StateMachine.add('BEGIN_AUTO', auto_state.BeginAuto(),
									 					 transitions={'to_teleop':'TELEOP',
									 												'to_simulation':'SIMULATION',
									 												'to_auto':'SUCCEEDED'})
			smach.StateMachine.add('TELEOP', teleop_state.Teleop(),
									 					 transitions={'complete':'BEGIN_AUTO'})
			smach.StateMachine.add('SIMULATION', sim_state.Simulation(),
									 					 transitions={'complete':'SUCCEEDED'})
			
			
		smach.StateMachine.add('SETUP_TREE', setup_sm,
													 transitions={'SUCCEEDED':'END'})
										
	
	outcome = bt.execute()
	
if __name__ == '__main__':
	main()
	# SHOULD IT TERMINATE ITSELF AFTER COMPLETION
	rospy.spin()
