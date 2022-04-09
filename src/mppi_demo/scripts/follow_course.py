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
from mppi_controller.msg import FollowCourseAction, FollowCourseGoal
from geometry_msgs.msg import PoseStamped
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
    args, _ = parser.parse_known_args()
    return args

def random_if_none(x):
    """ Return the value itself if it's not None, else a random number.
    """
    if x is not None:
        return x
    return random.uniform(4.5, 6.5)

if __name__ == "__main__":
    # initialize node
    rospy.init_node("follow_course")

    # parse input arguments
    args = parse_args()

    # create action client
    client = actionlib.SimpleActionClient(args.server_name, FollowCourseAction)
    rospy.loginfo("Waiting (indefinitely) for action server {}".format(args.server_name))
    client.wait_for_server()

    # create publisher for marker
    marker_pub = rospy.Publisher(args.marker_topic, Marker, queue_size=1, latch=True)

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

    rospy.loginfo("Sending goal: \n{}".format(goal))
    client.send_goal(goal)
    marker_pub.publish(marker)

    # wait for success (or failure)
    client.wait_for_result()
    rospy.loginfo("Server succeeded! (I think)")

    # and, we're done
