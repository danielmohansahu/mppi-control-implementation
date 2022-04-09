#!/usr/bin/env python3
""" Fit a given dataset via Regression.
"""

# STL
import os
import argparse

# Data Science
import pandas
import seaborn as sns
from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt

# nominal location of full extracted data
DEFAULT_FILENAME = os.path.abspath(os.path.join(__file__, "..", "..", "data", "collated_runs.csv"))

# independent variables
INDEPENDENT_VARS = ["wheel_radius", "wheel_separation", "slip_left", "slip_right"]

# dependent variable(s)
DEPENDENT_VARS = ["cost"]

if __name__ == "__main__":
    # parse args

    # load csv w/ pandas
    dataframe_full = pandas.read_csv(DEFAULT_FILENAME)

    # get the subset of columns we really care about
    dataframe = dataframe_full[ INDEPENDENT_VARS + DEPENDENT_VARS ]

    import code
    code.interact(local=locals())
