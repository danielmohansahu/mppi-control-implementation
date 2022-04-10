#!/usr/bin/env python3
""" Plot performance of a given bag file.

Displays:
    - Trajectory (desired and achieved).
    - Commanded steer / throttle
    - Velocity odometry
"""

# STL
import argparse

# MatPlotLib
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

# Custom
from model_regression.extract import Extractor
from model_regression.cost import cost_function

# required bag topics
TOPICS = {
    "params": "/jackal/mppi_controller/parameter_updates",
    "goal": "/jackal/follow_course/goal",
    "result": "/jackal/follow_course/result",
    "odom": "/jackal/odom",
    "cmd": "/jackal/cmd_vel",
}

def parse_args():
    parser = argparse.ArgumentParser("Plot the results of a given run.")
    parser.add_argument("bag", help="The bag file to process.")
    args,_ = parser.parse_known_args()
    return args

def plot_results(filename, data):
    """ Construct a figure showing the things we care about.
    """
    # extract data
    T0 = data.odom[0].header.stamp.to_sec()
    T = [msg.header.stamp.to_sec() - T0 for msg in data.odom]
    X = [msg.pose.pose.position.x for msg in data.odom]
    Y = [msg.pose.pose.position.y for msg in data.odom]
    VX = [msg.twist.twist.linear.x for msg in data.odom]
    CT = [msg.linear.x for msg in data.cmd]
    CS = [msg.angular.z for msg in data.cmd]

    # initialize axes
    fig,axs = plt.subplots(3)
    fig.suptitle("Results for '{}'\n Cost: {:.2f}".format(filename, data.cost))

    ### first subplot is XY performance vs. desired trajectory
    
    # goal trajectory
    axs[0].add_patch(Ellipse((0,0), width=2*data.goal.goal.major, height=2*data.goal.goal.minor, fill=False, label="Desired"))
    # achieved trajectory
    axs[0].plot(X, Y, label="Achieved")

    ### second subplot is linear velocty vs. desired
    axs[1].plot(T, VX, label="Achieved")
    axs[1].plot([T[0],T[-1]], [data.goal.goal.velocity, data.goal.goal.velocity], "black", label="Desired")

    ### third subplot is commanded twist / steer
    #@TODO! need timestamps.

    ### common operations
    axs[0].grid()
    axs[0].legend()
    axs[1].grid()
    axs[1].legend()
    axs[2].grid()
    axs[2].legend()

if __name__ == "__main__":
    # process arguments
    args = parse_args()

    # extract data from bag
    extractor = Extractor(params_topic=TOPICS["params"],
                          goal_topic=TOPICS["goal"],
                          result_topic=TOPICS["result"],
                          odom_topic=TOPICS["odom"],
                          cmd_topic=TOPICS["cmd"],
                          cost_function=cost_function)
    data = extractor.extract(args.bag)

    # construct plots
    plot_results(args.bag.split("/")[-1], data)

    # show, then exit
    plt.show()

