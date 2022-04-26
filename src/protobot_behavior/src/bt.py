#!/usr/bin/env python3.8

import rospy
import smach
import smach_ros

from states import startup_state, teleop_state, led_array, nav_states, search_state, prep_nav_state


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
                transitions={"complete" : "BEGIN_MISSION"})
            smach.StateMachine.add(
                "BEGIN_MISSION", startup_state.BeginMission(),
                transitions={"to_teleop" : "TELEOP",
                             "to_auto" : "SUCCESS"})
            smach.StateMachine.add(
                "TELEOP", teleop_state.Teleop(),
                transitions={"complete" : "BEGIN_AUTO"})

        smach.StateMachine.add(
            "SETUP_TREE", setup_sm,
            transitions={"SUCCESS" : "PREP_NAV"})

        # Transition to navigation
        smach.StateMachine.add(
            "PREP_NAV", prep_nav_state.PrepNav(),
            transitions={"to_nav" : "NAV_TREE",
                         "finish" : "MISSION_END"})
        
        # NAVIGATION SM
        nav_sm = smach.StateMachine(outcomes=["SUCCESS", "FAIL"])
        with nav_sm:
            smach.StateMachine.add(
                "WAIT_FOR_GOAL", nav_states.WaitForGoal(),
                transitions={"ready" : "NAV_TO_GOAL"})
            smach.StateMachine.add(
                "NAV_TO_GOAL", nav_states.NavToGoal(),
                transitions={"reached_goal" : "FLASH_GREEN",
                             "start_search" : "SEARCH",
                             "aborted_goal" : "FAIL"})
            smach.StateMachine.add(
                "SEARCH", search_state.Search(),
                transitions={"goal_found" : "NAV_TO_GOAL")
            smach.StateMachine.add(
                "FLASH_GREEN", led_array.FlashGreen(),
                transitions={"complete" : "SUCCESS"})
        
        smach.StateMachine.add(
            "NAV_TREE", nav_sm,
            transitions={"SUCCESS" : "PREP_NAV"})

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