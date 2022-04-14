#!/usr/bin/python3
""" follow_course.py

A simple utility script to execute trajectory following.
"""

# STL
import random
import argparse

# ROS
import rospy
import actionlib
import dynamic_reconfigure.client
from mppi_controller.msg import FollowCourseAction, FollowCourseGoal
from mppi_controller.cfg import MPPIOptionsConfig as Options
from visualization_msgs.msg import Marker

def parse_args():
    """ Parse input arguments.
    """
    parser = argparse.ArgumentParser("Specify Goal arguments.")
    parser.add_argument("-n", "--server-name", default="jackal/follow_course",
                        help="Name of the action server to look for.")
    parser.add_argument("-m", "--marker-topic", default="jackal/goal_marker",
                        help="Name of the marker topic on which to publish.")
    parser.add_argument("-f", "--frame-id", default="jackal/odom",
                        help="Coordinate frame of the goal.")
    parser.add_argument("-x", "--x", default=None, type=float,
                        help="Major Axis size (m).")
    parser.add_argument("-y", "--y", default=None, type=float,
                        help="Minor Axis size (m).")
    parser.add_argument("-v", "--velocity", default=2.0, type=float,
                        help="Target linear velocity (m/s).")
    parser.add_argument("-d", "--duration", default=10.0, type=float,
                        help="Desired length of time to run.")
    parser.add_argument("-o", "--optimal", action="store_true",
                        help="Use (precomputed) optimal parameters.")
    parser.add_argument("-r", "--reconfigure-node", default="jackal/mppi_controller",
                        help="Node name to connect via dynamic_reconfigure.")
    args, _ = parser.parse_known_args()
    return args

def random_if_none(x):
    """ Return the value itself if it's not None, else a random number.
    """
    if x is not None:
        return x
    return random.uniform(4.5, 6.5)

def follow(args, client, pub):
    """ Core logic for sending a single course goal.
    """
    # create goal
    goal = FollowCourseGoal()
    goal.major = random_if_none(args.x)
    goal.minor = random_if_none(args.y)
    goal.velocity = args.velocity
    goal.duration = args.duration

    # create marker
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = args.frame_id
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.scale.x = 2.0 * goal.major
    marker.scale.y = 2.0 * goal.minor
    marker.scale.z = 0.01
    marker.pose.orientation.w = 1.0
    marker.color.g = 1.0
    marker.color.a = 0.25

    rospy.loginfo("Sending goal:")
    for param in ["major", "minor", "velocity", "duration"]:
        rospy.loginfo("\t{}: {:.3f}".format(param, getattr(goal,param)))
    client.send_goal(goal)
    pub.publish(marker)

    # wait for success (or failure)
    client.wait_for_result()
    rospy.loginfo("Server succeeded! (I think)")

    # is there a way to verify failure?
    return True

def set_optimal_params(args):
    """ Tell the robot to use optimized parameters.
    """
    # connect to reconfigure client
    rospy.loginfo("Waiting (indefinitely) for dynamic reconfigure client {}".format(args.reconfigure_node))
    reconfigure_client = dynamic_reconfigure.client.Client(args.reconfigure_node)

    # define params
    params = {
        "wheel_radius": 0.089,
        "wheel_separation": 0.556,
        "slip_left": 0.068,
        "slip_right": 0.061,
        "icr": 0.028
    }

    # send params
    reconfigure_client.update_configuration(params)
    rospy.sleep(1)
    print("Optimal params set.")

if __name__ == "__main__":
    # initialize node
    rospy.init_node("follow_course")

    # parse input arguments
    args = parse_args()

    # use optimal params, if desired
    if args.optimal:
        set_optimal_params(args)

    # create action client
    client = actionlib.SimpleActionClient(args.server_name, FollowCourseAction)
    rospy.loginfo("Waiting (indefinitely) for action server {}".format(args.server_name))
    client.wait_for_server()

    # create publisher for marker
    marker_pub = rospy.Publisher(args.marker_topic, Marker, queue_size=1, latch=True)

    # send and track goal
    follow(args, client, marker_pub)

    # and, we're done
