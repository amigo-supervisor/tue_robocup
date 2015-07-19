#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy
import itertools
import os

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fast_simulator_object_spawner')

    W = client.SimWorld()

    env = os.environ['ROBOT_ENV']

    if env == "robotics_testlabs":

        # Testlab B: Position of the shelf: 3.18; 2.713; 0.35
        models = ["coke", "milk", "fanta"]
        X = [3.10, 3.12]
        Y = [2.56, 2.71, 2.86]
        Z = [0.75]

        for index, (model, x,y,z) in enumerate(list(itertools.product(models, X, Y, Z))[:6]):
            W.add_object(model+"-"+str(index), model, x, y, z)

    if env == "rwc2015":

        # Right_bookcase
        W.add_object("coke11", "sim-coke", 1.15, -8.13, 1.10)
        W.add_object("coke12", "sim-coke", 1.15, -8.13, 0.75)
        W.add_object("coke13", "sim-coke", 1.15, -8.33, 0.75)
        W.add_object("coke14", "sim-coke", 1.15, -8.7,  0.75)


