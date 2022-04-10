""" cost.py
Defines a cost function for evaluating the performance of a given bagged run.

"""

# STL
import math
import statistics

# SCIPY
from scipy import optimize

class Ellipse():
    """ Convenience class to calculate deviance from desired trajectory.
    """
    def __init__(self, major, minor):
        self.major = major
        self.minor = minor

    def dist(self, x, y):
        """ Return euclidean distance from self.
        """
        # exploit symmetry to consider one quadrant
        x, y = abs(x), abs(y)
        x0, y0 = self.closest_point(x, y)
        return math.sqrt( (x - x0) ** 2.0 + (y - y0) ** 2.0 )

    def closest_point(self, x, y):
        """ Find the closest point on the ellipse.
        
        Notes:
            Assumes (X,Y) are non-negative
            Ignoring issues with small numbers by assuming  << X, Y ~= 0
            Assumes center is (0,0)

        https://math.stackexchange.com/questions/90974/calculating-distance-of-a-point-from-an-ellipse-border
        """
        # handle edge cases
        if x < 1e-3 and y < 1e-3:
            # we're on the origin; return closest
            return (self.major, 0) if (self.major < self.minor) else (0, self.minor)

        # non-origin or cardinal axis point
        t = optimize.root(self.construct_F(x, y), [0])
        if not t.success:
            print("Unable to solve for closest point to ({},{}), ignoring. \nError: {}".format(x, y, t.message))
            return (x,y)
        # calculate closest point
        x0 = self.major * self.major * x / (self.major ** 2.0 - t.x)
        y0 = self.minor * self.minor * y / (self.minor ** 2.0 - t.x)
        return (x0, y0)

    def construct_F(self, x, y):
        """ Construct optimizable function. We want the root.

        https://math.stackexchange.com/questions/90974/calculating-distance-of-a-point-from-an-ellipse-border
        """
        m, n = self.major, self.minor
        F = lambda t: ( (m * x * (n ** 2.0 - t)) ** 2.0 + (n * y * (m ** 2.0 - t)) ** 2.0 - ((m ** 2.0 - t) * (n ** 2.0 - t)) ** 2.0 )
        return F

def cost_function(goal, result, odom, cmd):
    """ Apply known heuristics to assign cost to the given run."

    Cost is comprised of:
     euclidean deviance from desired trajectory (accuracy)
     + deviance from desired linear velocity (accuracy)
     + control variance (smoothness)

    Analysis is bounded by the timestamps between [goal, result]
    """
    
    # extract start / end times
    start = goal.header.stamp
    end = result.header.stamp

    # instantiate helper class
    ellipse = Ellipse(goal.goal.major, goal.goal.minor)

    # initialize cost
    cumulative_cost = 0.0

    # add in cost associated with deviation from desired trajectory and velocity
    cost = 0.0
    for msg in odom:
        if msg.header.stamp < start:
            # message received before goal sent; ignore
            continue
        elif msg.header.stamp > end:
            # we're done; stop
            break
        # add cost for position deviation
        cost += ellipse.dist(msg.pose.pose.position.x, msg.pose.pose.position.y)
        # add cost for velocity deviation
        cost += abs(goal.goal.velocity - msg.twist.twist.linear.x)
    cumulative_cost += cost / len(odom)

    # add in controller stddev cost
    #  note that we don't restrict timestamps here; hopefully not a big deal?
    cumulative_cost += statistics.stdev([msg.linear.x for msg in cmd])
    cumulative_cost += statistics.stdev([msg.angular.z for msg in cmd])

    # done!
    return cumulative_cost
