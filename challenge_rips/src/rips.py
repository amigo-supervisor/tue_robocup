#!/usr/bin/python
import roslib;
import rospy
import smach
import sys

from robot_smach_states.util.designators import EdEntityDesignator
import robot_smach_states as states

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_rips')
STARTING_POINT = challenge_knowledge.starting_point
EXIT_1 = challenge_knowledge.exit_1
EXIT_2 = challenge_knowledge.exit_2
EXIT_3 = challenge_knowledge.exit_3

def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:

        # Start challenge via StartChallengeRobust
        smach.StateMachine.add( "START_CHALLENGE_ROBUST",
                                    states.StartChallengeRobust(robot, STARTING_POINT, use_entry_points = True),
                                    transitions={   "Done":"GO_TO_INTERMEDIATE_WAYPOINT",
                                                    "Aborted":"GO_TO_INTERMEDIATE_WAYPOINT",
                                                    "Failed":"GO_TO_INTERMEDIATE_WAYPOINT"})   # There is no transition to Failed in StartChallengeRobust (28 May)

        # smach.StateMachine.add('GO_TO_INTERMEDIATE_WAYPOINT',
        #                             states.NavigateToObserve(robot, EdEntityDesignator(robot, id="rips1"), radius=0.7),
        #                             transitions={   'arrived':'ASK_CONTINUE',
        #                                             'unreachable':'ASK_CONTINUE',
        #                                             'goal_not_defined':'ASK_CONTINUE'})

        smach.StateMachine.add('GO_TO_INTERMEDIATE_WAYPOINT',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id="rips1"), radius=0.7),
                                    transitions={   'arrived':'ASK_CONTINUE',
                                                    'unreachable':'GO_TO_INTERMEDIATE_WAYPOINT_BACKUP',
                                                    'goal_not_defined':'GO_TO_INTERMEDIATE_WAYPOINT_BACKUP'})

        smach.StateMachine.add('GO_TO_INTERMEDIATE_WAYPOINT_BACKUP',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id="rips1"), radius=0.7),
                                    transitions={   'arrived':'ASK_CONTINUE',
                                                    'unreachable':'ASK_CONTINUE',
                                                    'goal_not_defined':'ASK_CONTINUE'})

        smach.StateMachine.add("ASK_CONTINUE",
                        states.AskContinue(robot),
                        transitions={   'continue':'SAY_CONTINUEING',
                                        'no_response':'SAY_CONTINUEING'})

        smach.StateMachine.add( 'SAY_CONTINUEING',
                                states.Say(robot, ["I heard continue, so I will move to the exit now. See you guys later!"], block=False),
                                transitions={'spoken':'GO_TO_EXIT'})

        # Amigo goes to the exit (waypoint stated in knowledge base)
        smach.StateMachine.add('GO_TO_EXIT',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id=EXIT_1), radius = 1.2),
                                    transitions={   'arrived':'AT_END',
                                                    'unreachable':'GO_TO_EXIT_2',
                                                    'goal_not_defined':'GO_TO_EXIT_2'})

        smach.StateMachine.add('GO_TO_EXIT_2',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id=EXIT_2), radius = 0.5),
                                    transitions={   'arrived':'AT_END',
                                                    'unreachable':'GO_TO_EXIT_3',
                                                    'goal_not_defined':'GO_TO_EXIT_3'})

        smach.StateMachine.add('GO_TO_EXIT_3',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id=EXIT_3), radius = 0.5),
                                    transitions={   'arrived':'AT_END',
                                                    'unreachable':'RESET_ED_TARGET',
                                                    'goal_not_defined':'AT_END'})

        smach.StateMachine.add('RESET_ED_TARGET',
                                states.ResetED(robot),
                                transitions={   'done'              :   'GO_TO_EXIT_1'})

        # Finally amigo will stop and says 'goodbye' to show that he's done.
        smach.StateMachine.add('AT_END',
                                states.Say(robot, "Goodbye"),
                                transitions={'spoken':'Done'})
    return sm


############################## initializing program ##############################
if __name__ == '__main__':
    rospy.init_node('rips_exec')

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "[CHALLENGE RIPS] Please provide robot name as argument."
        exit(1)

    states.util.startup(setup_statemachine, robot_name=robot_name)
