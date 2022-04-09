#!/usr/bin/env python3
""" Extract all data from the given set of bags into CSV.
"""

# STL
import os
from glob import glob
import argparse

# TQDM
from tqdm import tqdm

# Custom
from model_regression.extract import Extractor
from model_regression.cost import cost_function

# expected topics within bag
TOPICS = {
    "params": "/jackal/mppi_controller/parameter_updates",
    "goal": "/jackal/follow_course/goal",
    "result": "/jackal/follow_course/result",
    "odom": "/jackal/odom",
    "cmd": "/jackal/cmd_vel",
}

# desired output location for collated data
DEFAULT_OUTPUT = os.path.abspath(os.path.join(__file__, "..", "..",  "data", "collated_runs.csv"))

# nominal location of where data gets stored
DEFAULT_DATA_DIRECTORY = os.path.abspath(os.path.join(__file__, "..", "..", "..", "mppi_demo", "data"))

def parse_args():
    parser = argparse.ArgumentParser("Extract bagged data from the target directory into CSV.")
    parser.add_argument("--directory", default=DEFAULT_DATA_DIRECTORY,
                        help="Location of bagged data.")
    parser.add_argument("-o", "--output", default=DEFAULT_OUTPUT,
                        help="Location to store CSV data.")
    args,_ = parser.parse_known_args()
    return args

if __name__ == "__main__":
    # parse input arguments
    args = parse_args()

    # instantiate data extraction class
    extractor = Extractor(params_topic=TOPICS["params"],
                          goal_topic=TOPICS["goal"],
                          result_topic=TOPICS["result"],
                          odom_topic=TOPICS["odom"],
                          cmd_topic=TOPICS["cmd"],
                          output=args.output,
                          cost_function=cost_function)

    # get list of bags to consider
    bags = glob(os.path.join(args.directory, "*.bag"))
    bags.sort()

    print("Extracting data from {} bags found in \n{}.".format(len(bags), args.directory))
    error_count = 0
    for bag in tqdm(bags):
        if not extractor.extract(bag):
            error_count += 1

    print("Finished data extraction! {}/{} successful.".format(len(bags) - error_count, len(bags)))



