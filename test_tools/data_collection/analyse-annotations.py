#! /usr/bin/env python

# System
import os

# TU/e Robotics
from robocup_knowledge import knowledge_loader


# Colors from printing on screen
class BColors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


def count_images(objects, path):
    """ Counts the images in the subdirectories of 'path'. The subdirectories are identified by the provided objects.
    The results are printed to screen

    :param objects: list with strings
    :param path: string indicating the path
    """
    # List the number of occurrences in each sub folder
    ustats = []  # Unverified
    for o in objects:
        p = os.path.join(path, o)

        # If path doesn't exist, we probably don't have any images
        if not os.path.exists(p):
            ustats.append((o, 0))
        else:
            ustats.append((o, len(os.listdir(p))))

    # Sort and print the results
    ustats.sort(key=lambda tup: tup[1], reverse=True)
    for s in ustats:
        if s[1] > 0:
            print "{}: {}".format(s[0], s[1])
        else:
            print BColors.WARNING + "{}: {}".format(s[0], s[1]) + BColors.ENDC

    # Sanity check: try to identify mismatches between object names and annotated images
    print BColors.BOLD + "\nPossible mismatches:" + BColors.ENDC
    print "Annotated but not in knowledge"
    for candidate in os.listdir(path):
        if candidate not in objects:
            print BColors.WARNING + candidate + BColors.ENDC

    print "\n"


if __name__ == "__main__":

    # Get the names of the objects in which we are interested
    common_knowledge = knowledge_loader.load_knowledge("common")
    objects = common_knowledge.object_names

    # Get the path to the folder where images are stored
    robot_env = os.environ.get("ROBOT_ENV")
    path = os.path.join(os.path.expanduser("~"), "MEGA", "data", robot_env, "training_data", "annotated")

    # Count both verified and unverified
    for v in ["unverified", "verified"]:
        tpath = os.path.join(path, v)
        print BColors.HEADER + BColors.BOLD + "{}:".format(v) + BColors.ENDC
        count_images(objects=objects, path=tpath)
