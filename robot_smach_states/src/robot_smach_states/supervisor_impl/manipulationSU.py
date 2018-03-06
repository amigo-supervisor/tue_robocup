# import PyKDL as kdl
from robot_skills.util.entity import Entity
from robot_skills.arms import Arm

import rospy
import smach
import tf
from robot_smach_states.manipulation.grasp_point_determination\
    import GraspPointDeterminant

from robot_skills.util.kdl_conversions import point_msg_to_kdl_vector, \
    FrameStamped, kdl_frame_to_pose_msg, VectorStamped
from robot_smach_states.util.designators import check_type
from robot_smach_states.manipulation import ResetOnFailure

# TODO:
#   grabSU
#     - gewoon grab gebruiken (maar dan zonder navigatetograsp)
#     - checken of het lukt met de moveit planning scene en kinect
#       obstructies live te detecteren
#     - mocht dit allemaal niet lukken, omdat grasp precompute erin zit
#       moet ik het allemaal zelf gaan schrijven maar dat wordt heel veel werk
#   moveArmOWS
#     - check if entity is in ws otherwise move arm in direction
#
#   - change arms
#   - add replan action
#   - call external c lib

# We gaan het volgende doen:
# - bewegen met arm naar een pose
# - grab als object in ws
# - gripper open dicht class
# - get planningscene collision shit from grasprecompute
# - eventueel graspprecompute niet gebruiken maar eigen moveit impl\
#
# 0. checkPathValid (while loop write to CIF)
# 0. isPathFinished (while loop write to CIF)
# 0. itemInWS (while loop write to CIF)
# 1. grabojectid
#    - pick arm closest to object and reserve that for that object action
# 2. check if visible (look at object)
# 3. check if in WS
#   yes then GrabSU
#   no, but inside outer range
#     if no obstacles on interpolated path base
#       then MoveArmSU and convert point to in WS
# 4. GrabSU:
#    - without gripper
#    - replan action
# 5. MoveArmSu
#    - check if object comes in WS (loop)
#    - replan action


class CheckObjectInWS(smach.state):  # better call this init
  def __init__(self, robot, arm, item):
    smach.State.__init__(self, outcomes=['inWS', 'notInWS'])

  def execute(self):
    # call CIF to ask for edge inWS
    # call CIF to ask for edge notInWS
    return 'succes'


class MoveArmToObjectSU(smach.state):
  # make this state machine with MoveArmSU
  def __init__(self, robot, arm, item):
    smach.Sate.__init__(self, outcomes=['succeeded', 'failed', 'objectInWS',
                                        'replan'])
    self.robot = robot
    self.item = item
    self.arm = arm

  def execute(self):
    # convert item to pos and send goal
    goal_map = VectorStamped(0, 0, 0, frame_id=self.item.id)
    goal_bl = goal_map.projectToFrame(self.robot.robot_name + '/base_link',
                                      tf_listener=self.robot.tf_listener)
    if goal_bl is None:
        return 'failed'
    else:
        return 'failed'

    # Offset so goal_bl is in WS
    # tfToWSOffset = find ws from base_link and find the difference
    goal_bl.vector[0] = goal_bl.vector.x() - tfToWSOffset

    if not self.arm.send_goal(goal_bl, timeout=0):  # maybe register callbacks when
      return('failed')                         # movement finished? not waiting

    # while loop checking for edges
    #   call CIF to ask for edge objectInWS
    #     cancel goal arm
    #   call CIF to ask for edge succeeded (movement finished)
    #   call CIF to ask edge replan (pathvalid)
    #     cancel all goals
    #     restart state


class MoveArmSU(smach.state):
  def __init__(self, robot, arm, frameStamped, timeout=20,
               pre_grasp=False):
    """
    Move arm to FrameStamped pos.
    :param robot: robot to execute state with
    :param arm: Designator that resolves to arm to grab with.
                E.g. UnoccupiedArmDesignator
    :param pose:
    """
    smach.State.__init__(self, outcomes=['succes', 'failed'])

    self.robot = robot
    self.arm = arm
    self.frameStamped = frameStamped
    self.pre_grasp = pre_grasp
    self.timeout = timeout

  def execute(self, userdata=None):
    if self.arm.send_goal(self.frameStamped,
                          timeout=self.timeout,
                          pre_grasp=self.pre_grasp):
      return 'succes'
    else:
      return 'failed'


# Copy of PrepareEdGrasp but without opening gripper
class PrepareEdGraspSU(smach.State):
  def __init__(self, robot, arm, grab_entity):
    """
    Set the arm in the appropriate position before actually grabbing
    :param robot: robot to execute state with
    :param arm: Designator that resolves to arm to grab with.
                E.g. UnoccupiedArmDesignator
    :param grab_entity: Designator that resolves to the entity to grab.
                        E.g EntityByIdDesignator
    """
    smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                         output_keys=['arm'])

    # Assign member variables
    self.robot = robot
    self.arm_designator = arm
    self.grab_entity_designator = grab_entity

    check_type(grab_entity, Entity)

  def execute(self, userdata):
    arm = self.arm_designator.resolve()
    if not arm:
        rospy.logerr("Could not resolve arm")
        return "failed"
    userdata.arm = arm.side

    entity = self.grab_entity_designator.resolve()
    if not entity:
        rospy.logerr("Could not resolve grab_entity")
        return "failed"

    # Torso up (non-blocking)
    self.robot.torso.high()

    # Arm to position in a safe way
    arm.send_joint_trajectory('prepare_grasp', timeout=0)

    # Make sure the head looks at the entity
    self.robot.head.look_at_point(
        VectorStamped(vector=entity._pose.p, frame_id="/map"), timeout=0.0)

    return 'succeeded'


class PickUpSU(smach.State):
  def __init__(self, robot, arm, grab_entity):
    """
    Pick up an item given an arm and an entity to be picked up
    :param robot: robot to execute this state with
    :param arm: Designator that resolves to the arm to grab the grab_entity
                with. E.g. UnoccupiedArmDesignator
    :param grab_entity: Designator that resolves to the entity to grab.
                        e.g EntityByIdDesignator
    """
    smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                         output_keys=['arm'])

    # Assign member variables
    self.robot = robot
    self.arm_designator = arm

    check_type(grab_entity, Entity)
    self.grab_entity_designator = grab_entity
    self._gpd = GraspPointDeterminant(robot)

  def execute(self, userdata):

    grab_entity = self.grab_entity_designator.resolve()
    if not grab_entity:
      rospy.logerr("Could not resolve grab_entity")
      return "failed"

    arm = self.arm_designator.resolve()
    if not arm:
      rospy.logerr("Could not resolve arm")
      return "failed"
    # Using userdata makes sure we don't need to do any more arm entity magic
    userdata.arm = arm.side

    goal_map = VectorStamped(0, 0, 0, frame_id=grab_entity.id)

    try:
      # Transform to base link frame
      goal_bl = goal_map.projectToFrame(self.robot.robot_name +
                                        '/base_link',
                                        tf_listener=self.robot.tf_listener)
      if goal_bl is None:
        rospy.logerr('Transformation of goal to base failed')
        return 'failed'
    except tf.Exception, tfe:
      rospy.logerr('Transformation of goal to base failed: {0}'.format(tfe))
      return 'failed'

    # Make sure the torso and the arm are done
    self.robot.torso.wait_for_motion_done(cancel=True)
    arm.wait_for_motion_done(cancel=True)

    # This is needed because the head is not entirely still when the
    # look_at_point function finishes
    rospy.sleep(rospy.Duration(0.5))

    # Update the entity (position)
    # segm_res = self.robot.ed.update_kinect("%s" % grab_entity.id)

    # Resolve the entity again because we want the latest pose
    updated_grab_entity = self.grab_entity_designator.resolve()

    rospy.loginfo("ID to update: {0}".format(grab_entity.id))
    if not updated_grab_entity:
      rospy.logerr("Could not resolve the updated grab_entity, "
                   "this should not happen [CHECK WHY THIS IS HAPPENING]")
      grab_entity = self.associate(original_entity=grab_entity)
    else:
      rospy.loginfo("Updated pose of entity (dx, dy, dz) : (%f, %f, %f)" %
                    (updated_grab_entity.pose.frame.p.x() -
                     grab_entity.pose.frame.p.x(),
                     updated_grab_entity.pose.frame.p.y() -
                     grab_entity.pose.frame.p.y(),
                     updated_grab_entity.pose.frame.p.z() -
                     grab_entity.pose.frame.p.z()))
      grab_entity = updated_grab_entity

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # Grasp point determination
    grasp_framestamped = self._gpd.get_grasp_pose(grab_entity, arm)

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    goal_map = VectorStamped(0, 0, 0, frame_id=grab_entity.id)

    # try:
    # In case grasp point determination didn't work
    if not grasp_framestamped:
      goal_bl = goal_map.projectToFrame(self.robot.robot_name + '/base_link',
                                        tf_listener=self.robot.tf_listener)
      if goal_bl is None:
        return 'failed'
      else:
        return 'failed'
    else:
      # We do have a grasp pose, given as a kdl frame in map
      if self.robot.tf_listener. \
              waitForTransform("/map", self.robot.robot_name + "/base_link"):
        # Transform to base link frame
        goal_bl = grasp_framestamped. \
            projectToFrame(self.robot.robot_name + "/base_link",
                           tf_listener=self.robot.tf_listener)
        if goal_bl is None:
          return 'failed'
      else:
        return 'failed'

    # Grasp
    rospy.loginfo('Start grasping')
    if not arm.send_goal(goal_bl,
                         timeout=20, pre_grasp=True,
                         allowed_touch_objects=[grab_entity.id]
                         ):
      self.robot.speech.speak('I am sorry but I cannot move my \
         arm to the object position', block=False)
      rospy.logerr('Grasp failed')
      arm.reset()
      # arm.send_gripper_goal('close', timeout=0.0)
      return 'failed'
      # Close gripper
      # arm.send_gripper_goal('close')

      arm.occupied_by = grab_entity

      # Lift
      arm.send_joint_goal('carrying_pose', timeout=0.0)

      # Check if the object is present in the gripper
      if arm.object_in_gripper_measurement.is_empty:
        # If state is empty, grasp has failed
        result = "failed"
        rospy.logerr("Gripper is not holding an object")
        self.robot.speech.speak("Whoops, something went terribly wrong")
        arm.occupied_by = None  # Set the object the arm is holding to None
      else:
        # State is holding, grasp succeeded.
        # If unknown: sensor not there, assume gripper is holding
        # and hope for the best
        result = "succeeded"
        if arm.object_in_gripper_measurement.is_unknown:
          rospy.logwarn("GripperMeasurement unknown")

      # Reset head
      self.robot.head.cancel_goal()

      return result


class GrabSU(smach.state):
  def __init__(self, robot, item, arm=None):
    """
    Let the given robot move to an entity and grab that entity using some arm.
    Supervisory implementation with arm reservation and choosing.
    :param robot: Robot to use
    :param item: Designator that resolves to the item to grab.
                 E.g. EntityByIdDesignator
    :param arm: Designator that resolves to the arm to use for grabbing.
                Eg. UnoccupiedArmDesignator
    :return:
    """
    smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

    # Check types or designator resolve types
    check_type(item, Entity)
    if arm is not None:
      check_type(arm, Arm)
      if arm.locked:
        rospy.logwarn('Prefered arm in use picking other arm')
        arm = pickClosestArm(item)
    else:
      arm = pickClosestArm(item)

    if arm is None:
        rospy.logerr('No free arm for grabSU')
        return 'failed'

    with self:
      smach.StateMachine.add('CHECK_IN_WORKSPACE',  # better call this init
                             CheckObjectInWS(robot, arm, item),
                             transitions={'inWS': 'PREPARE_GRASP',
                                          'notInWs': 'MOVE_ARM',
                                          'failed': 'RESET_FAILURE'})

      smach.StateMachine.add('MOVE_ARM_TO_OBJECT',
                             MoveArmToObjectSU(robot, arm, item),
                             transitions={'succes'})

      smach.StateMachine.add('PREPARE_GRASP',
                             PrepareEdGraspSU(robot, arm, item),
                             transitions={'succeeded': 'NAVIGATE_TO_GRAB',
                                          'failed': 'RESET_FAILURE'})

      smach.StateMachine.add('GRAB', PickUpSU(robot, arm, item),
                             transitions={'succeeded': 'done',
                                          'failed': 'RESET_FAILURE'})

      smach.StateMachine.add("RESET_FAILURE", ResetOnFailure(robot),
                             transitions={'done': 'failed'})
