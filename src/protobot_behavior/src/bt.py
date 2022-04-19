#!/usr/bin/env python3.8

import rospy
import smach
import smach_ros

from states import auto_state, teleop_state, simulation_state as sim_state, red_led_state, check_leg_state, led_array, wait_for_goal_state, nav_to_goal_state, search_state


def main():
    rospy.init_node("smach_state_machine")

    # ROOT SM
    bt = smach.StateMachine(outcomes=["MISSION_END"])
    with bt:

        # SETUP SM
        setup_sm = smach.StateMachine(outcomes=["SUCCESS"])
        with setup_sm:
            
            # Prompt if teleop should start
            setup_sm.userdata.start_teleop = input("BEGIN TELEOP? (y)\n")

            smach.StateMachine.add(
                "FLASH_RED", led_array.FlashRed(),
                transitions={"complete" : "CHECK_LEG"})

			smach.StateMachine.add(
				"CHECK_LEG", check_leg_state.CheckLeg(),
                transitions={"done" : "BEGIN_AUTO"})

            smach.StateMachine.add(
                "BEGIN_AUTO", auto_state.BeginAuto(),
                transitions={"to_teleop" : "TELEOP",
                             "to_auto" : "SUCCESS"})
            smach.StateMachine.add(
                "TELEOP", teleop_state.Teleop(),
                transitions={"complete" : "BEGIN_AUTO"})

        smach.StateMachine.add("SETUP_TREE", setup_sm,
            transitions={"SUCCESS" : "NAV_TREE"})

        # NAVIGATION SM
        nav_sm = smach.StateMachine(outcomes=["SUCCESS", "FAIL"])
        with nav_sm:
            smach.StateMachine.add(
                "WAIT_FOR_GOAL", wait_for_goal_state.WaitForGoal(),
                transitions={"ready" : "NAV_TO_GOAL"})
            smach.StateMachine.add(
                "NAV_TO_GOAL", nav_to_goal_state.NavToGoal(),
                transitions={"reached_goal" : "SUCCESS",
                             "start_search" : "SEARCH",
                             "abort_path" : "FAIL"})
            smach.StateMachine.add(
                "SEARCH", search_state.Search(),
                transitions={"goal_found" : "NAV_TO_GOAL"})
        
        smach.StateMachine.add("NAV_TREE", nav_sm,
            transitions={"SUCCESS" : "MISSION_END"})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('Introspection', bt, '/SM_ROOT')
    sis.start()

    # Execute state machine
    outcome = bt.execute()

    # rosrun smach_viewer smach_viewer.py


if __name__ == "__main__":
    main()
    # SHOULD IT TERMINATE ITSELF AFTER COMPLETION
    rospy.spin()
    sis.stop()