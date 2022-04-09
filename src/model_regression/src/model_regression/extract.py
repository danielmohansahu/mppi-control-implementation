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
from collections import OrderedDict

# ROS
import rosbag

class Extractor:
    """ Object Oriented class for extracting parameters + costs from a given bag.
    """
    def __init__(self, params_topic, goal_topic, result_topic, odom_topic, cmd_topic, output, cost_function):
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

        # initialize output file
        self.output = output
        if os.path.exists(output):
            print("Overriding existing output file '{}'".format(output))
        with open(output, "w") as csvfile:
            pass

    def extract(self, filepath):
        """ Extract relevant data from the given bag file and write to output (CSV).
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

        # get total cost from the other topics
        cost = self.cost_function(goal=data["goal"][0], result=data["result"][0], odom=data["odom"], cmd=data["cmd"])

        # write to file
        self.write(filepath.split("/")[-1], params, cost)

        # indicate success
        return True

    def write(self, filename, params, cost):
        """ Write a new line to the given output file.

        We assume the given params are in the correct order.
        """
        first_time = os.path.getsize(self.output) == 0

        with open(self.output, "a") as csvfile:
            writer = csv.writer(csvfile, delimiter=",")

            # write header iff this is our first line
            if first_time:
                header = ["filename"] + list(params.keys()) + ["cost"]
                writer.writerow(header)

            writer.writerow([filename] + list(params.values()) + [cost])


