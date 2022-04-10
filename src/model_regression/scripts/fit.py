#!/usr/bin/env python3
""" Fit a given dataset via Regression.

Notes:
    Plot pairwise via 'sns.pairplot(df)'
"""

# STL
import os
import argparse

# Plotting
import seaborn as sns
import matplotlib.pyplot as plt

# Data Science
import pandas
from sklearn.linear_model import LinearRegression
from sklearn.neighbors import LocalOutlierFactor
from sklearn import metrics, model_selection

# nominal location of full extracted data
DEFAULT_FILENAME = os.path.abspath(os.path.join(__file__, "..", "..", "data", "collated_runs.csv"))

# independent variables
INDEPENDENT_VARS = ["wheel_radius", "wheel_separation", "slip_left", "slip_right"]

# dependent variable(s)
DEPENDENT_VAR = "cost"

def parse_args():
    parser = argparse.ArgumentParser("Attempt to fit a model to collated data from bagged runs.")
    parser.add_argument("-f", "--filename", default=DEFAULT_FILENAME,
                        help="File containing collected data.")
    args,_ = parser.parse_known_args()
    return args

def fit_linear_model(df, features = INDEPENDENT_VARS, target = DEPENDENT_VAR, evaluate = True):
    """ Attempt to fit a linear model to the given data.

    Returns:
        function: A predictive function based on the regressed model.

    Great help from:
    https://datagy.io/python-sklearn-linear-regression/
    https://machinelearningmastery.com/model-based-outlier-detection-and-removal-in-python/
    """
    # initialize model and data
    model = LinearRegression()

    # perform outlier rejection on target variable
    #  @TODO think this through; we could have cost nonlinearities...
    #  This implictly assumes that poor cost performance is a result of issues other than the parameters assigned
    lof = LocalOutlierFactor()
    yhat = lof.fit_predict( df[[target]] )
    mask = yhat != -1

    # get rid of bad points
    X = df[features][mask]
    y = df[target][mask]

    # if in evaluation mode, first try to train on a subset of the data
    if evaluate:
        # Splitting the data into training and testing
        X_train, X_test, y_train, y_test = model_selection.train_test_split(X, y, shuffle=True, train_size=0.3)

        # fit model
        model.fit(X_train, y_train)

        # evaluate
        predictions = model.predict(X_test)
        print("Linear Model Performance:")
        print("\t R2: \t{}".format(metrics.r2_score(y_test, predictions)))
        print("\t RMSE: \t{}".format(metrics.mean_squared_error(y_test, predictions, squared=False)))

    # we want to return a model trained against the full set of data
    model.fit(X, y)

    # convert to a predictive function which converts arguments from 'features' into 'target'
    def predict(*args):
        target = model.intercept_
        for i,a in enumerate(*args):
            target += a * model.coef_[i]
        return target
    return predict

if __name__ == "__main__":
    # parse args
    args = parse_args()

    # sanity check file exists and is non-empty
    if not os.path.exists(args.filename) or os.path.getsize(args.filename) == 0:
        raise RuntimeError("Given empty or null data file '{}'. Did you remember to call 'extract'?".format(args.filename))

    # load csv w/ pandas
    dataframe_full = pandas.read_csv(args.filename)

    # get the subset of columns we really care about
    df = dataframe_full[ INDEPENDENT_VARS + [DEPENDENT_VAR] ]

    # attempt to fit a linear regression model:
    function = fit_linear_model(df, INDEPENDENT_VARS, DEPENDENT_VAR)

    import code
    code.interact(local=locals())
