#!/usr/bin/env python3
""" Run a parametric data collect on the 'follow course' 

This script will (continuously, until stopped) send 'follow_course'
goals to the concurrently running MPPI server.

The procedure of each run is as follows:
    1. Verify connectivity.
    2. Begin bag collection.
    3. Update relevant controller parameters via dynamic reconfigure.
    4. Send goal with hardcoded duration.
    5. Wait for goal completion (success or failure)
    6. Save and close bag.
"""

# STL
import os
import random
import signal
import subprocess
from datetime import datetime
from contextlib import contextmanager

# ROS
import rospy
import rosbag
import actionlib
import dynamic_reconfigure.client
from mppi_controller.cfg import MPPIOptionsConfig as Options
from mppi_controller.msg import FollowCourseAction, FollowCourseGoal
from visualization_msgs.msg import Marker

# custom
from follow_course import follow, parse_args

# Global variables (yuck)
DURATION = 120
VELOCITY = 2.0
RECONFIGURE_NODE = "jackal/mppi_controller"
BAG_DEST = os.path.abspath(os.path.join(__file__, "..", "..", "data"))
PARAMS = ["wheel_radius", "wheel_separation", "slip_left", "slip_right", "icr"]
TOPICS = ["/tf", "/tf_static", "/clock", "/jackal/cmd_vel", "/jackal/odom",
          "/jackal/follow_course/goal", "/jackal/follow_course/result",
          "/jackal/mppi_controller/parameter_descriptions", "/jackal/mppi_controller/parameter_updates"]

def generate_bag_name():
    """ Generate a randomized bag name.
    """
    return "run_" + datetime.now().strftime("%Y-%m-%d-%H-%M-%S")

@contextmanager
def bag_record(name):
    """ Context manager to ensure bags get closed properly.

    Proper destruction of subprocesses from:
    https://stackoverflow.com/questions/4789837/how-to-terminate-a-python-subprocess-launched-with-shell-true
    """
    # start bagging
    full_path = os.path.join(BAG_DEST, name)
    rospy.loginfo("Saving bag to {}".format(full_path))
    cmd = "rosbag record --lz4 -O {} {}".format(full_path, " ".join(TOPICS))
    rospy.logdebug("Bagging via \n\t'{}'".format(cmd))
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
    try:
        yield
    finally:
        # stop bag
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)

def execute():
    """ Full end-to-end execution of a single goal.
    """
    # get and update default arguments
    args = parse_args()
    args.duration = DURATION
    args.velocity = VELOCITY

    # randomly permute parameters
    params = {}
    for param in PARAMS:
        params[param] = random.uniform(Options.min[param], Options.max[param])

    # create dynamic reconfigure client
    rospy.loginfo("Waiting (indefinitely) for dynamic reconfigure client {}".format(RECONFIGURE_NODE))
    reconfigure_client = dynamic_reconfigure.client.Client(RECONFIGURE_NODE)

    # create publisher for marker
    marker_pub = rospy.Publisher(args.marker_topic, Marker, queue_size=1, latch=True)

    # construct client and publisher
    client = actionlib.SimpleActionClient(args.server_name, FollowCourseAction)
    rospy.loginfo("Waiting (indefinitely) for action server {}".format(args.server_name))
    client.wait_for_server()

    # once action server is up we can start bagging
    with bag_record(generate_bag_name()):
        # wait a bit to make sure the bag opened properly
        rospy.sleep(1.0)

        # send a dynamic reconfigure request
        reconfigure_client.update_configuration(params)
        rospy.loginfo("Set random params:")
        for param,val in params.items():
            rospy.loginfo("\t{}: {:.3f}".format(param, val))

        # wait, again, just to make sure things are ok
        rospy.sleep(1.0)

        # execute goal
        follow(args, client, marker_pub)

if __name__ == "__main__":
    # initialize ROS node
    rospy.init_node('follow_course_data_collection')

    # continue sending goals until stop is requested
    while not rospy.is_shutdown():
        try:
            execute()
        except KeyboardInterrupt:
            print("Stop requested! Last run might be corrupted...")
