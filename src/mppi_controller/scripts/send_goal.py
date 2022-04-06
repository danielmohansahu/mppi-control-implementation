#!/usr/bin/python3
""" send_goal.py

A simple utility script to execute a goal.
"""

# STL
import argparse

# ROS
import rospy
from geometry_msgs.msg import PoseStamped

def parse_args():
    """ Parse input arguments.
    """
    parser = argparse.ArgumentParser("Specify Goal arguments.")
    parser.add_argument("-t", "--topic", default="jackal/goal",
                        help="Name of the topic on which to publish.")
    parser.add_argument("-f", "--frame-id", default="jackal/odom",
                        help="Coordinate frame of the goal.")
    parser.add_argument("-x", "--x", default=0.0, type=float,
                        help="X location of the goal.")
    parser.add_argument("-y", "--y", default=0.0, type=float,
                        help="Y location of the goal.")
    
    args, _ = parser.parse_known_args()
    return args

if __name__ == "__main__":
    # initialize node
    rospy.init_node("send_goal")

    # parse input arguments
    args = parse_args()

    # create publisher
    publisher = rospy.Publisher(args.topic, PoseStamped, queue_size=1)

    # create goal
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = args.frame_id
    goal.pose.position.x = args.x
    goal.pose.position.y = args.y

    # sleep a tiny bit to make sure everything is connected. Rospy is weird.
    rospy.sleep(0.5)

    rospy.loginfo("Sending goal: \n{}".format(goal))
    publisher.publish(goal)

    # and, we're done.
