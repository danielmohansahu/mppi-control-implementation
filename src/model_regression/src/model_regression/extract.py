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

# ROS
import rosbag

class Extractor:
    """ Object Oriented class for extracting parameters + costs from a given bag.
    """
    def __init__(self, params, params_topic, goal_topic, result_topic, odom_topic, cmd_topic, output, cost_function):
        # expected independent parameters
        #  sort to make sure we write this in the correct order later!
        self.params = params
        self.params.sort()

        # expected topics for param / cost extraction
        self.topics = {
            "params": params_topic,
            "goal": goals_topic,
            "result": results_topic,
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
        if not os.file.exists(filepath):
            raise RuntimeError("Given non-existent filename '{}'".format(filename))

        # load bag metadata
        bag = rosbag.Bag(filepath, "r")
        types, topics_info = bag.get_type_and_topic_info()

        # check that all expected topics exist
        for topic in self.topics.values():
            if topic not in topics_info:
                raise RuntimeError("Bag '{}' does not contain required topic '{}' !")

        # iterate through the bag, collecting relevant topics
        data = {key: [] for key in self.topics.keys()}
        for topic,msg,_ in bag.read_messages(topics=list(self.topics.keys())):
            data[topic].append(msg)

        # sanity check number of topics
        for item in ["params", "goal", "result"]:
            topic = self.topics[item]
            if len(data[topic]) != 1:
                print("Unexpected number of messages for topic '{}'".format(topic))
                import pdb;pdb.set_trace()

        # @TODO extract from message
        import pdb;pdb.set_trace()
        params = []

        # get total cost from the other topics
        cost = self.cost_function(goal=self.topics["goal"][0], result=self.topics["result"][0], odom=self.topics["odom"], cmd=self.topics["cmd"])

        # write to file
        write(filepath.split("/")[-1], params, cost)


    def write(self, filename, params, cost):
        """ Write a new line to the given output file.

        We assume the given params are in the correct order.
        """
        with open(self.output, "a") as csvfile:
            writer = csv.writer(csvfile, delimeter=",")
            writer.writerow([filename] + params + [cost])


