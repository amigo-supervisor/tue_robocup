#! /usr/bin/python

# ------------------------------------------------------------------------------------------------------------------------
# By Jorrit Smit 2018
# ------------------------------------------------------------------------------------------------------------------------

import sys
import rospy
import smach

import hmi

from cb_planner_msgs_srvs.msg import PositionConstraint

import robot_smach_states as states

from robot_smach_states.navigation import NavigateToObserve, NavigateToWaypoint, NavigateToSymbolic
from robot_smach_states.util.designators import EntityByIdDesignator

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_supervisor')


print "=============================================="
print "==           CHALLENGE SUPERVISOR           =="
print "=============================================="

def setup_statemachine(robot):
  sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:
      smach.StateMachine.add('INITIALIZE',
                               states.Initialize(robot),
                               transitions={'initialized':    'SET_INITIAL_POSE',
                                            'abort':          'Aborted'})

      smach.StateMachine.add('SET_INITIAL_POSE',
                             states.SetInitialPose(robot, challenge_knowledge.starting_point),
                             transitions={'done': 'WAIT_TO_BEGIN',
                                          "preempted": 'Aborted',
                                          'error': 'Aborted'})

      smach.StateMachine.add('WAIT_TO_BEGIN',
                            states.Say(robot,["Waiting to begin"], block=True),
                            transitions={'spoken' : 'ASK_TO_BEGIN'})
      smach.StateMachine.add('ASK_TO_BEGIN',
                            )




############################## initializing program ##############################
if __name__ == '__main__':
    rospy.init_node('supervisor_exec')

    states.util.startup(setup_statemachine, challenge_name="supervisor")