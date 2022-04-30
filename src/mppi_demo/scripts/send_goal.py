#!/usr/bin/python3
""" send_goal.py

A simple utility script to execute a goal.
"""

# STL
import random
import argparse

# ROS
import rospy
import actionlib
import dynamic_reconfigure.client
from mppi_controller.msg import WaypointAction, WaypointGoal
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

def parse_args():
    """ Parse input arguments.
    """
    parser = argparse.ArgumentParser("Specify Goal arguments.")
    parser.add_argument("-n", "--server-name", default="jackal/waypoint",
                        help="Name of the action server to look for.")
    parser.add_argument("-m", "--marker-topic", default="jackal/goal_marker",
                        help="Name of the marker topic on which to publish.")
    parser.add_argument("-f", "--frame-id", default="jackal/odom",
                        help="Coordinate frame of the goal.")
    parser.add_argument("-x", "--x", default=None, type=float,
                        help="X location of the goal.")
    parser.add_argument("-y", "--y", default=None, type=float,
                        help="Y location of the goal.")
    parser.add_argument("-r", "--radius", default=1.0, type=float,
                        help="Radius of goal.")
    parser.add_argument("-v", "--velocity", default=2.0, type=float,
                        help="Target linear velocity (m/s).")
    parser.add_argument("-t", "--timeout", default=0.0, type=float,
                        help="Timeout for the goal; values <= 0.0 will be interpreted as infinity.")
    parser.add_argument("-o", "--optimal", action="store_true",
                        help="Use (precomputed) optimal parameters.")
    parser.add_argument("-rn", "--reconfigure-node", default="jackal/mppi_controller",
                        help="Node name to connect via dynamic_reconfigure.")
    
    args, _ = parser.parse_known_args()
    return args

def random_if_none(x):
    """ Return the value itself if it's not None, else a random number.
    """
    if x is not None:
        return x
    return 10.0 * (random.random() - 0.5)

def set_optimal_params(args):
    """ Tell the robot to use optimized parameters.
    """
    # connect to reconfigure client
    rospy.loginfo("Waiting (indefinitely) for dynamic reconfigure client {}".format(args.reconfigure_node))
    reconfigure_client = dynamic_reconfigure.client.Client(args.reconfigure_node)

    # define params
    params = {
        "wheel_radius": 0.150,
        "wheel_separation": 0.614,
        "slip_left": 0.057,
        "slip_right": 0.054,
        "icr": 0.023
    }

    # send params
    reconfigure_client.update_configuration(params)
    rospy.sleep(1)
    print("Optimal params set.")

if __name__ == "__main__":
    # initialize node
    rospy.init_node("send_goal")

    # parse input arguments
    args = parse_args()

    # use optimal params, if desired
    if args.optimal:
        set_optimal_params(args)

    # create action client
    client = actionlib.SimpleActionClient(args.server_name, WaypointAction)
    rospy.loginfo("Waiting (indefinitely) for action server {}".format(args.server_name))
    client.wait_for_server()

    # create publisher for marker
    marker_pub = rospy.Publisher(args.marker_topic, Marker, queue_size=1, latch=True)

    # create goal
    goal = WaypointGoal()
    goal.pose.header.stamp = rospy.Time.now()
    goal.pose.header.frame_id = args.frame_id
    goal.pose.pose.position.x = random_if_none(args.x)
    goal.pose.pose.position.y = random_if_none(args.y)
    goal.velocity = args.velocity
    goal.radius = args.radius
    goal.timeout = args.timeout

    # create marker
    marker = Marker()
    marker.header = goal.pose.header
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.scale.x = 2.0 * args.radius
    marker.scale.y = 2.0 * args.radius
    marker.scale.z = 0.01
    marker.pose.position.x = goal.pose.pose.position.x
    marker.pose.position.y = goal.pose.pose.position.y
    marker.pose.orientation.w = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    rospy.loginfo("Sending goal: \n{}".format(goal))
    client.send_goal(goal)
    marker_pub.publish(marker)

    # wait for success (or failure)
    client.wait_for_result()
    result = client.get_result()

    rospy.loginfo("Server {}".format("succeeded!" if result.success else "failed."))

    # and, we're done
