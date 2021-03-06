#!/usr/bin/env python3
""" Fit a given dataset via Regression.
"""

# STL
import os
import argparse

# Plotting
import seaborn as sns
import matplotlib.pyplot as plt

# Data Science
import pandas
import numpy as np
import skopt
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn import metrics, model_selection

from sklearn.covariance import EllipticEnvelope
from sklearn.neighbors import LocalOutlierFactor
from sklearn.ensemble import IsolationForest

# Custom
from mppi_controller.cfg import MPPIOptionsConfig as Options

# ignore some false positive warnings
pandas.options.mode.chained_assignment = None  # default='warn'

# nominal location of full extracted data
DEFAULT_FILENAME = os.path.abspath(os.path.join(__file__, "..", "..", "data", "collated_runs.csv"))

# independent variables
INDEPENDENT_VARS = ["wheel_radius", "wheel_separation", "slip_left", "slip_right", "icr"]

# dependent variable(s)
DEPENDENT_VARS = ["cost", "log_cost", "pos_cost", "vel_cost", "ctrl_cost"]
DEFAULT_DEPENDENT_VAR = "cost"

def parse_args():
    parser = argparse.ArgumentParser("Attempt to fit a model to collated data from bagged runs.")
    parser.add_argument("-f", "--filename", default=DEFAULT_FILENAME,
                        help="File containing collected data.")
    parser.add_argument("-p", "--plot", action="store_true",
                        help="Plot relationships between features.")
    parser.add_argument("-l", "--log", action="store_true",
                        help="Apply natural logarithm to targer.")
    parser.add_argument("-d", "--dependent_variable", type=str, default=DEFAULT_DEPENDENT_VAR,
                        help="Target variable to use in regression. Default is '{}', options are {}.".format(
                            DEFAULT_DEPENDENT_VAR, DEPENDENT_VARS))
    args,_ = parser.parse_known_args()
    return args

def fit_model(df, outliers, features = INDEPENDENT_VARS, target = DEFAULT_DEPENDENT_VAR, evaluate = True):
    """ Attempt to fit a model to the given data.

    Returns:
        function: A predictive function based on the regressed model.

    Great help from:
    https://datagy.io/python-sklearn-linear-regression/
    https://machinelearningmastery.com/model-based-outlier-detection-and-removal-in-python/
    https://data36.com/polynomial-regression-python-scikit-learn/
    """

    # initialize model and data
    poly = PolynomialFeatures(degree=2, include_bias=False)
    model = LinearRegression()
    # get rid of bad points and fit to a polynomial
    mask = (outliers == False)
    X = poly.fit_transform(df[features][mask])
    y = df[target][mask]

    # if in evaluation mode, first try to train on a subset of the data
    if evaluate:
        # Splitting the data into training and testing
        X_train, X_test, y_train, y_test = model_selection.train_test_split(X, y, shuffle=True, train_size=0.3)

        # fit model
        model.fit(X_train, y_train)

        # evaluate
        predictions = model.predict(X_test)
        print("  Model Performance:")
        print("\t R2: \t{}".format(metrics.r2_score(y_test, predictions)))
        print("\t RMSE: \t{}".format(metrics.mean_squared_error(y_test, predictions, squared=False)))
    else:
        # we want to return a model trained against the full set of data
        model.fit(X, y)

    # convert to a predictive function which converts arguments from 'features' into 'target'
    def predict(*args):
        return model.predict(poly.fit_transform(np.array(args).reshape(1,-1)))[0]
    return predict

if __name__ == "__main__":
    # parse args
    args = parse_args()

    # sanity check file exists and is non-empty
    if not os.path.exists(args.filename) or os.path.getsize(args.filename) == 0:
        raise RuntimeError("Given empty or null data file '{}'. Did you remember to call 'extract'?".format(args.filename))

    # load csv w/ pandas
    print("Loading data from '{}'...".format(args.filename))
    dataframe_full = pandas.read_csv(args.filename)

    # get the subset of columns we really care about
    df = dataframe_full[ INDEPENDENT_VARS + [args.dependent_variable] ]

    # apply natural logarithm, if desired
    if args.log:
        df[args.dependent_variable] = np.log(df[args.dependent_variable])

    # perform outlier rejection on target variable
    lof = IsolationForest(contamination=0.025)
    yhat = lof.fit_predict( df[[args.dependent_variable]] )
    outliers = (yhat == -1)

    # plot, if desired
    if args.plot:
        # pairwise plot of raw data
        df_plot = df.copy()
        df_plot["outliers"] = outliers
        sns.pairplot(df_plot, hue="outliers")
        plt.show()

    # attempt to fit a regression model:
    #  also perform evaluation mode, for reference
    print("Fitting multivariate Polynomial model...")
    _ = fit_model(df, outliers, INDEPENDENT_VARS, args.dependent_variable, True)
    function = fit_model(df, outliers, INDEPENDENT_VARS, args.dependent_variable, False)

    # use bayesian optimization to find the "optimal" parameters
    print("Finding 'optimal' parameters to minimize cost...")
    bounds = [(Options.min[param], Options.max[param]) for param in INDEPENDENT_VARS]
    res = skopt.gp_minimize(function, bounds)

    print("Found 'optimal' parameters:\n\t" + "\n\t".join(["{}: {}".format(INDEPENDENT_VARS[i], res.x[i]) for i in range(len(INDEPENDENT_VARS))]))

    # show regression lines against raw data
    if args.plot:
        # pairwise plot
        df_plot = df.copy()
        df_plot["outliers"] = outliers
        grid = sns.pairplot(df_plot, hue="outliers")

        # iterate through component axes
        for ax in grid.figure.axes:
            xlabel,ylabel = ax.get_xlabel(), ax.get_ylabel()

            # skip axes that aren't plotting our target in Y against something interesting
            if ylabel != args.dependent_variable or xlabel == ylabel or xlabel == "outliers":
                continue

            # reduce our optimized function to 1D, replacing other args
            #  with their optimal value
            idx = INDEPENDENT_VARS.index(xlabel)
            def f(x):
                return function(*[v if i != idx else x for i,v in enumerate(res.x) ])

            # plot this modified function
            X = np.linspace(*ax.get_xlim())
            Y = [f(x) for x in X]
            ax.plot(X, Y, 'k')

        plt.show()

