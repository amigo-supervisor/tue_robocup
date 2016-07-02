#!/usr/bin/env python

import rospy, sys, robot_smach_states, random
from robot_skills.sergio import Sergio as Robot

if __name__ == "__main__":
    rospy.init_node('challenge_open_sergio')

    robot = Robot()

    robot.speech.speak("Here is Sergio! Let's show some navigation!", block=False)
    robot.speech.speak("My knowledge is based on AMIGO's view of the world!", block=False)
    robot.base.force_drive(0.25, 0, 0, 5.0)  # x, y, z, time in seconds

    exclude_list = ["amigo", "sergio", "walls", "floor", "root"]

    ids = [e.id for e in robot.ed.get_entities() if e.has_shape and not any(ex in e.id for ex in exclude_list)]
    random.shuffle(ids)

    print "IDS:", ids

    for id in ids:

        robot.speech.speak("I am going to navigate to the %s" % id, block=False)

        machine = robot_smach_states.NavigateToSymbolic(robot, {robot_smach_states.util.designators.EntityByIdDesignator(robot, id=id): "in_front_of"},
                                                        robot_smach_states.util.designators.EntityByIdDesignator(robot, id=id))

        result = machine.execute()

        if result == "arrived":
            robot.speech.speak("I arrived at the %s" % id, block=True)
        else:
            robot.speech.speak("I failed to navigate to the %s" % id, block=True)
