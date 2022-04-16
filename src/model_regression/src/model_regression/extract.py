""" extract.py
Load a suite of bag files and extract salient information to CSV.

This essentially looks for expected topics and parameters. The former
are used in conjunction with cost.py to create a single dependent
variable 'cost' for each run, while the latter are extracted as
the driving independent variables.
"""

# STL
import os
import csv
import math
from collections import OrderedDict
from dataclasses import dataclass

# ROS
import rosbag

# Custom
from .cost import CostResult

class Extractor:
    """ Object Oriented class for extracting parameters + costs from a given bag.
    """

    @dataclass
    class Results:
        """ Convenience class to output results.
        """
        params: ...                 # state of all parameters
        goal: ...                   # goal of this run
        result: ...                 # results of this run
        odom: list                  # list of odometry messages
        cmd: list                   # list of control messages
        costs: CostResult = None    # estimated costs

    def __init__(self, params_topic, goal_topic, result_topic, odom_topic, cmd_topic, cost_function, output=None, grace_period=10):
        # expected topics for param / cost extraction
        self.topics = {
            "params": params_topic,
            "goal": goal_topic,
            "result": result_topic,
            "odom": odom_topic,
            "cmd": cmd_topic
        }

        # save cost function
        self.cost_function = cost_function

        # amount of time to allow for initialization
        self.grace_period = grace_period

        # initialize output file
        self.output = output
        if output is not None:
            if os.path.exists(output):
                print("Overriding existing output file '{}'".format(output))
            with open(output, "w") as csvfile:
                pass

    def extract(self, filepath):
        """ Extract relevant data from the given bag file.
        """

        # perform sanity checks on bag
        if not os.path.exists(filepath):
            print("\tgiven non-existent filename '{}'".format(filename))
            return False

        # load bag metadata
        bag = rosbag.Bag(filepath, "r")
        types, topics_info = bag.get_type_and_topic_info()

        # check that all expected topics exist
        for topic in self.topics.values():
            if topic not in topics_info:
                print("\tbag '{}' does not contain required topic '{}' !".format(filepath, topic))
                return False

        # iterate through the bag, collecting relevant topics
        data = {key: [] for key in self.topics.keys()}
        for topic,msg,_ in bag.read_messages():
            if topic not in self.topics.values():
                continue
            key = next(k for k,t in self.topics.items() if t == topic)
            data[key].append(msg)

        # sanity check number of topics
        if len(data["params"]) != 2:
            print("\t unexpected number of messages for topic '{}' in '{}'".format(self.topics["params"], filepath))
            return False

        for item in ["goal", "result"]:
            topic = self.topics[item]
            if len(data[item]) != 1:
                print("\tunexpected number of messages for topic '{}' in '{}'".format(topic, filepaht))
                return False

        # @TODO extract from message
        unsorted_params = []
        for type_ in ["bools", "ints", "strs", "doubles"]:
            for val in getattr(data["params"][-1], type_):
                unsorted_params.append((val.name, val.value))
        unsorted_params.sort()
        params = OrderedDict({k:v for k,v in unsorted_params})

        # initialize Results data
        res = self.Results(
            params=params,
            goal=data["goal"][0],
            result=data["result"][0],
            odom=data["odom"],
            cmd=data["cmd"])

        # get total cost from the other topics
        res.costs = self.cost_function(res.goal, res.result, res.odom, res.cmd)

        # return collected information
        return res

    def to_csv(self, filepath):
        """ Wrapper script to extract data and write to a CSV

        We assume the given params are in the correct order.
        """
        if self.output is None:
            raise RuntimeError("Call to 'to_csv' for an Extractor class with no specified 'output' file!")

        # extract data
        res = self.extract(filepath)

        # sanity check we succeeded
        if not res:
            return False

        # if we got this far we have good data; write
        first_time = os.path.getsize(self.output) == 0
        with open(self.output, "a") as csvfile:
            writer = csv.writer(csvfile, delimiter=",")

            # write header iff this is our first line
            if first_time:
                header = ["filename"] + list(res.params.keys()) + ["cost", "log_cost", "pos_cost", "vel_cost", "ctrl_cost"]
                writer.writerow(header)

            writer.writerow([filepath.split("/")[-1]] + list(res.params.values()) + [res.costs.cost, res.costs.log, res.costs.pos, res.costs.vel, res.costs.ctr])

        # indicate success
        return True


