[![Build Status](https://travis-ci.org/tue-robotics/tue_robocup.svg?branch=master)](https://travis-ci.org/tue-robotics/tue_robocup)

# ADDED FUNCTIONALITY FROM THIS REPO
challenge_supervisor is added that implements the CIF synthesised supervisor on the robot. It acts as a wrapper around the supervisor library and executes the proper actions depending on the events fired by the supervisor. The Amigo_bringup repo adds a launch file, so the wrapper can be started on the robot

# tue_robocup
RoboCup challenge implementations
This is to high-level code that TU Eindhoven's AMIGO robot uses to perform RoboCup@Home challenges.

Most, if not all, challenges are implemented as (large) hierarchical state machines, using the robot_smach_states library of states/state machines.
These states use the robot_skills robot abstraction layer.
