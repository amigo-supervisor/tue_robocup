#! /usr/bin/python2

# ------------------------------------------------------------------------------------------------------------------------
# By Jorrit Smit 2018
# ------------------------------------------------------------------------------------------------------------------------

# import sys
import rospy
import smach

# import hmi

# from cb_planner_msgs_srvs.msg import PositionConstraint

import robot_smach_states as states

# from robot_smach_states.navigation import NavigateToObserve, NavigateToWaypoint, NavigateToSymbolic
# from robot_smach_states.util.designators import EntityByIdDesignator

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_supervisor')


print "=============================================="
print "==           CHALLENGE SUPERVISOR           =="
print "=============================================="


class MoveArm(smach.State):
  def __init__(self, robot):
    smach.State.__init__(self, outcomes=['succes', 'failed'])

  def execute(self, userdata=None):
    # Call CIF lib from here

    return 'succes'


class MoveBase(smach.State):
  def __init__(self, robot, entity_designator_area_name_map, entity_lookat_designator):
    smach.State.__init__(self, outcomes=['succes', 'failed'])

  def execute(self, userdata=None):
    # Call CIF lib from here
    return 'succes'


class GrabObject(smach.State):
  def __init__(self, robot):
    smach.State.__init__(self, outcomes=['succes', 'failed'])

  def execute(self, userdata=None):
    # Call CIF lib from here

    return 'succes'


class ReleaseObject(smach.State):
  def __init__(self, robot):
    smach.State.__init__(self, outcomes=['succes', 'failed'])

  def execute(self, userdata=None):
    # Call CIF lib from here

    return 'succes'


# gets called when ANY child state terminates
def term_cb(outcome_map):
  if 'failed' not in outcome_map.viewvalues():
    return False
  else:
    return True


def setup_statemachine(robot):
  sm = smach.StateMachine(outcomes=['Done', 'Aborted'])

  with sm:
    smach.StateMachine.add('INITIALIZE',
                           states.Initialize(robot),
                           transitions={'initialized': 'SET_INITIAL_POSE',
                                        'abort': 'Aborted'})

    smach.StateMachine.add('GOTO_WAYPOINT_1',
                           states.NavigateToWaypoint(
                               robot, challenge_knowledge.starting_point),
                           transitions={'done': 'WAIT_TO_BEGIN',
                                        "preempted": 'Aborted',
                                        'error': 'Aborted'})

    smach.StateMachine.add('WAIT_TO_BEGIN',
                           states.AskContinue(robot, rospy.Duration(60)),
                           transitions={'continue': 'BEGIN_MOVEMENT',
                                        'no_response': 'WAIT_TO_BEGIN'})

    go_sm = smach.Concurrence(outcomes=['succes', 'iterate', 'failed'],
                              default_outcome='iterate',
                              outcome_map={'succes':
                              {'MOVE_ARM': 'succes',
                               'MOVE_BASE': 'succes',
                               'GRAB_OBJECT': 'succes'}},
                              child_termination_cb=term_cb)

    with go_sm:
      smach.Concurrence.add('MOVE_ARM', MoveArm(robot))
      smach.Concurrence.add('MOVE_BASE', MoveBase(robot, entity_designator_area_name_map, entity_lookat_designator))
      smach.Concurrence.add('GRAB_OBJECT', GrabObject(robot))

    smach.StateMachine.add('GO_SM', go_sm,
                           transitions={'iterate': 'GO_SM',
                                        'succes': 'PO_SM',
                                        'failed': 'Aborted'})

    po_sm = smach.Concurrence(outcomes=['succes', 'iterate', 'failed'],
                              default_outcome='iterate',
                              outcome_map={'succes':
                              {'MOVE_ARM': 'succes',
                               'MOVE_BASE': 'succes',
                               'RELEASE_OBJECT': 'succes'}},
                              child_termination_cb=term_cb)

    with po_sm:
      smach.Concurrence.add('MOVE_ARM', MoveArm(robot))
      smach.Concurrence.add('MOVE_BASE', MoveBase(robot, entity_designator_area_name_map, entity_lookat_designator))
      smach.Concurrence.add('RELEASE_OBJECT', ReleaseObject(robot))

    smach.StateMachine.add('PO_SM', po_sm,
                           transitions={'iterate': 'PO_SM',
                                        'succes': 'Done',
                                        'failed': 'Aborted'})
  return sm

# ############################# initializing program ############################## #
if __name__ == '__main__':
  rospy.init_node('supervisor_exec')

  states.util.startup(setup_statemachine, challenge_name="supervisor")
