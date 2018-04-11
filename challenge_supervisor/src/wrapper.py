#! /usr/bin/python2

# ------------------- #
# By Jorrit Smit 2018 #
# ------------------- #

from cif._amigo_engine import ffi, lib

from robot_smach_states.util.designators import Designator
import robot_smach_states.util.designators as ds

# from robot_smach_states import Place
from robot_smach_states import NavigateToSymbolic, getPlan, Grab, SegmentObjects

from robot_skills.util.robot_constructor import robot_constructor

from robot_skills.util.entity import Entity
from robot_skills.classification_result import ClassificationResult


import rospy
import smach


print "=============================================="
print "==           CHALLENGE SUPERVISOR           =="
print "=============================================="

arm_left_plan_viable = 0
arm_right_plan_viable = 0
base_plan_viable = 0
arm_left_collision_on_path = False
arm_left_moving_arm = False
arm_left_goal_reached = False
arm_left_object_in_arm = False
arm_right_collision_on_path = False
arm_right_moving_arm = False
arm_right_goal_reached = False
arm_right_object_in_arm = False
base_collision_on_path = False
base_moving_base = False
base_goal_reached = False
grab_skill_use_arm = 0
left_grab_exec_object_in_WS = 0
right_grab_exec_object_in_WS = 0
task_object_grabbed = False


def grab_skill_determine_grab_arm(pre):
  global grab_skill_use_arm
  grab_skill_use_arm = 2
  lib.grab_skill_use_arm_ = grab_skill_use_arm


def arm_left_idle(pre):
  global arm_left_plan_viable
  global arm_left_moving_arm
  robot.leftArm.cancel_goals
  arm_left_plan_viable = 0
  arm_left_moving_arm = False


def arm_left_move(pre):
  global arm_left_moving_arm
  global left_grab_exec_object_in_WS

  leftarm.resolve().send_goal('carrying_pose', timeout=0.0)
  arm_left_moving_arm = True
  # left_grab_exec_object_in_WS = 0
  Amigo_AssignInputVariables()


def arm_left_plan(pre):
  robot.leftArm.cancel_goals
  global arm_left_plan_viable
  arm_left_plan_viable = 1
  Amigo_AssignInputVariables()


def arm_left_replan(pre):
  robot.leftArm.cancel_goals
  global arm_left_plan_viable
  arm_left_plan_viable = 1
  Amigo_AssignInputVariables()


def arm_left_plan_to_grab(pre):
  robot.leftArm.cancel_goals
  global arm_left_plan_viable
  arm_left_plan_viable = 1
  Amigo_AssignInputVariables()


def arm_left_grab(pre):
  global arm_left_object_in_arm
  global task_object_grabbed
  global left_grab_exec_object_in_WS

  classification_result_designator = ds.VariableDesignator([], resolve_type=[ClassificationResult])

  # Inspect the waypoint and classify objects on it
  segmSM = SegmentObjects(robot, classification_result_designator.writeable, wp)
  status = segmSM.execute()

  grab_obj = ds.VariableDesignator(resolve_type=Entity)

  # take the first seen coke
  for obj in classification_result_designator.resolve():
    if obj.type == 'coke':
      grab_obj = ds.EntityByIdDesignator(robot, type=obj.id)
      break

  grabSM = Grab(robot, grab_obj, leftarm)
  status = grabSM.execute()
  if status == 'done':
    arm_left_object_in_arm = True
    task_object_grabbed = True
    # left_grab_exec_object_in_WS = 0
  else:
    arm_left_object_in_arm = False  # RETRY
    task_object_grabbed = False
  Amigo_AssignInputVariables()


def arm_right_idle(pre):
  global arm_right_plan_viable
  global arm_right_moving_arm
  robot.rightArm.cancel_goals
  arm_right_plan_viable = 0
  arm_right_moving_arm = False
  Amigo_AssignInputVariables()


def arm_right_move(pre):
  #   # convert item to pos and send goal
  # goal_map = VectorStamped(0, 0, 0, frame_id=self.item.id)
  # goal_bl = goal_map.projectToFrame(robot.robot_name + '/base_link',
  #                                   tf_listener=robot.tf_listener)
  # if goal_bl is None:
  #     return 'failed'
  # else:
  #     return 'failed'

  # # Offset so goal_bl is in WS
  # # tfToWSOffset = find ws from base_link and find the difference
  # goal_bl.vector.x(goal_bl.vector.x() - tfToWSOffset)

  # if not robot.rightArm.send_goal(goal_bl, timeout=0):
  #   return('failed')                   # when movement finished? not waiting
  global arm_right_moving_arm
  global right_grab_exec_object_in_WS

  rightarm.resolve().send_joint_goal('carrying_pose', timeout=0.0)
  arm_right_moving_arm = True
  # right_grab_exec_object_in_WS = 0
  Amigo_AssignInputVariables()


def arm_right_plan(pre):
  robot.rightArm.cancel_goals
  global arm_right_plan_viable
  arm_right_plan_viable = 1
  Amigo_AssignInputVariables()


def arm_right_replan(pre):
  robot.rightArm.cancel_goals
  global arm_right_plan_viable
  arm_right_plan_viable = 1
  Amigo_AssignInputVariables()


def arm_right_plan_to_grab(pre):
  robot.rightArm.cancel_goals
  global arm_right_plan_viable
  arm_right_plan_viable = 1
  Amigo_AssignInputVariables()


def arm_right_grab(pre):
  global arm_right_object_in_arm
  global task_object_grabbed
  global right_grab_exec_object_in_WS

  classification_result_designator = ds.VariableDesignator([], resolve_type=[ClassificationResult])
  grab_obj = ds.VariableDesignator(resolve_type=Entity)

  while grab_obj.resolve() == None:
    # Inspect the waypoint and classify objects on it
    segmSM = SegmentObjects(robot, classification_result_designator.writeable, wp)
    status = segmSM.execute()

    # take the first seen coke
    for obj in classification_result_designator.resolve():
      if obj.type == 'coke':
        grab_obj = ds.EntityByIdDesignator(robot, id=obj.id)
        break

  grabSM = Grab(robot, grab_obj, rightarm)
  status = grabSM.execute()
  if status == 'done':
    task_object_grabbed = True
    arm_right_object_in_arm = True
    # right_grab_exec_object_in_WS = 0
  else:
    task_object_grabbed = False
    arm_right_object_in_arm = False  # RETRY
  Amigo_AssignInputVariables()


def base_idle(pre):
  global base_moving_base
  global base_plan_viable
  robot.base.local_planner.cancelCurrentPlan()
  base_plan_viable = 0
  base_moving_base = False
  Amigo_AssignInputVariables()


def base_move(pre):
  global base_moving_base
  global base_plan_viable
  global wp
  global objects_to_navigate_ds
  global base_goal_reached

  base_goal_reached = False

  wp = objects_to_navigate_ds.pop()

  navSM = NavigateToSymbolic(robot, {wp: "in_front_of"}, wp)

  # Create a simple SMACH state machine 
  sm = smach.StateMachine(outcomes=['moving','unreachable','goal_not_defined', 'preempted'])
  # Open the container
  with sm:
  # Add states to the container
    smach.StateMachine.add('GET_PLAN',                          getPlan(robot, navSM.generateConstraint, True),
        transitions={'unreachable'                          :   'unreachable',
                     'goal_not_defined'                     :   'goal_not_defined',
                     'goal_ok'                              :   'moving',
                     'preempted'                            :   'preempted'})
  # Execute SMACH plan
  outcome = sm.execute()
  if outcome == 'moving':
    base_moving_base = True
  elif outcome == 'unreachable'or 'goal_not_defined':
    base_plan_viable = 2
    objects_to_navigate_ds.append(wp)
  Amigo_AssignInputVariables()


def base_plan(pre):
  global base_plan_viable
  base_plan_viable = 1
  Amigo_AssignInputVariables()


def left_grab_exec_start(pre):
  global grab_skill_use_arm
  grab_skill_use_arm = 0  # left arm is broken
  lib.grab_skill_use_arm_ = grab_skill_use_arm


def right_grab_exec_start(pre):
  global grab_skill_use_arm
  grab_skill_use_arm = 0  # left arm is broken
  lib.grab_skill_use_arm_ = grab_skill_use_arm


def left_grab_exec_check_in_WS(pre):
  global left_grab_exec_object_in_WS
  if base_goal_reached == True:
    left_grab_exec_object_in_WS = 2
  else:
    left_grab_exec_object_in_WS = 1
  lib.left_grab_exec_object_in_WS_ = left_grab_exec_object_in_WS
  print(left_grab_exec_object_in_WS)


def right_grab_exec_check_in_WS(pre):
  global right_grab_exec_object_in_WS
  if base_goal_reached == True:
    right_grab_exec_object_in_WS = 2
  else:
    right_grab_exec_object_in_WS = 1
  lib.right_grab_exec_object_in_WS_ = right_grab_exec_object_in_WS
  print(right_grab_exec_object_in_WS)


function_dispatcher = {'grab_skill_determine_grab_arm': grab_skill_determine_grab_arm,
'arm_left_idle': arm_left_idle,
'arm_left_plan': arm_left_plan,
'arm_left_move': arm_left_move,
'arm_left_replan': arm_left_replan,
'arm_left_plan_to_grab': arm_left_plan_to_grab,
'arm_left_grab': arm_left_grab,
'arm_right_idle': arm_right_idle,
'arm_right_move': arm_right_move,
'arm_right_plan': arm_right_plan,
'arm_right_replan': arm_right_replan,
'arm_right_plan_to_grab': arm_right_plan_to_grab,
'arm_right_grab': arm_right_grab,
'base_idle': base_idle,
'base_move': base_move,
'base_plan': base_plan,
'left_grab_exec_start': left_grab_exec_start,
'right_grab_exec_start': right_grab_exec_start,
'left_grab_exec_check_in_WS': left_grab_exec_check_in_WS,
'right_grab_exec_check_in_WS': right_grab_exec_check_in_WS}


@ffi.def_extern()
def Amigo_InfoEvent(event, pre):
  # rospy.loginfo("%s event arrived", ffi.string(lib.Amigo_event_names[event]))
  try:
      function = function_dispatcher[ffi.string(lib.Amigo_event_names[event])]
  except KeyError:
     rospy.loginfo("%s key error", ffi.string(lib.Amigo_event_names[event]))
     return
  if pre == False:
    rospy.loginfo("%s event fired", ffi.string(lib.Amigo_event_names[event]))
    function(pre)


@ffi.def_extern()
def Amigo_AssignInputVariables():
  global right_grab_exec_object_in_WS
  global left_grab_exec_object_in_WS
  global base_goal_reached
  status = robot.base.local_planner.getStatus()
  if status == "arrived":
    base_goal_reached = True

  left_grab_exec_object_in_WS = 0
  right_grab_exec_object_in_WS = 0
  lib.arm_left_plan_viable_ = arm_left_plan_viable
  # Input variable "int[0..2] arm_left.plan_viable". */
  lib.arm_left_collision_on_path_ = arm_left_collision_on_path
  # Input variable "bool arm_left.collision_on_path". */
  lib.arm_left_moving_arm_ = arm_left_moving_arm
  # Input variable "bool arm_left.moving_arm". */
  lib.arm_left_goal_reached_ = arm_left_goal_reached
  # Input variable "bool arm_left.goal_reached". */
  lib.arm_left_object_in_arm_ = arm_left_object_in_arm
  # Input variable "bool arm_left.object_in_arm". */
  lib.arm_right_plan_viable_ = arm_right_plan_viable
  # Input variable "int[0..2] arm_right.plan_viable". */
  lib.arm_right_collision_on_path_ = arm_right_collision_on_path
  # Input variable "bool arm_right.collision_on_path". */
  lib.arm_right_moving_arm_ = arm_right_moving_arm
  # Input variable "bool arm_right.moving_arm". */
  lib.arm_right_goal_reached_ = arm_right_goal_reached
  # Input variable "bool arm_right.goal_reached". */
  lib.arm_right_object_in_arm_ = arm_right_object_in_arm
  # Input variable "bool arm_right.object_in_arm". */
  lib.base_plan_viable_ = base_plan_viable
  # Input variable "int[0..2] base.plan_viable". */
  lib.base_collision_on_path_ = base_collision_on_path
  # Input variable "bool base.collision_on_path". */
  lib.base_moving_base_ = base_moving_base
  # Input variable "bool base.moving_base". */
  lib.base_goal_reached_ = base_goal_reached
  # Input variable "bool base.goal_reached". */
  lib.grab_skill_use_arm_ = grab_skill_use_arm
  # Input variable "bool grab_skill.use_arm". */
  lib.left_grab_exec_object_in_WS_ = left_grab_exec_object_in_WS
  # Input variable "bool left_grab_exec.object_in_WS". */
  lib.right_grab_exec_object_in_WS_ = right_grab_exec_object_in_WS
  # Input variable "bool right_grab_exec.object_in_WS". */
  lib.task_object_grabbed_ = task_object_grabbed
  # Input variable "bool task.object_grabbed". */


# ############################# initializing program ######################### #


if __name__ == '__main__':
  global objects_to_grab, leftarm, rightarm, coke, table1, table2, robot
  global objects_to_navigate_ds
  rospy.init_node('supervisor_wrapper', log_level=rospy.DEBUG)
  robot = robot_constructor("amigo")

  objects_to_grab = ["coke"]
  objects_to_navigate = ["hallway_table", "dinner_table"]
  objects_to_navigate_ds = []

  robot.base.set_initial_pose(0.2, 5, -1.4)  # not nescessary in demo extern mode

  leftarm = Designator(robot.leftArm)
  rightarm = Designator(robot.rightArm)

  objects_to_grab.reverse()  # Reverse here, because we are going to use list.pop()
  objects_to_navigate.reverse()  # Reverse here, because we are going to use list.pop()
  for item in objects_to_navigate:
    objects_to_navigate_ds.append(ds.EntityByIdDesignator(robot, id=item))

  raw_input('Press enter to continue: ')

  robot.base.set_initial_pose(0.2, 5, -1.4)  # not nescessary in demo extern mode

  lib.Amigo_EngineFirstStep()
  while not rospy.is_shutdown():
    lib.Amigo_EngineTimeStep(0.5)
    rospy.sleep(0.5)
