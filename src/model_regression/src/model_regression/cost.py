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

    def dist(self, x0, y0):
        """ Return euclidean distance from self.

        Taken from this very helpful Issue:
        https://github.com/0xfaded/ellipse_demo/issues/1
        """
        # exploit symmetry to consider one quadrant
        x0, y0 = abs(x0), abs(y0)

        # initialize first guesses
        tx,ty = 0.707,0.707

        for i in range(3):
            x = self.major * tx
            y = self.minor * ty

            ex = (self.major**2 - self.minor**2) * tx**3 / self.major
            ey = (self.minor**2 - self.major**2) * ty**3 / self.minor

            r = math.hypot(x - ex, y - ey)
            q = math.hypot(x0 - ex, y0 - ey)

            tx = min(1, max(0, ( (x0 - ex) * r / q + ex) / self.major))
            ty = min(1, max(0, ( (y0 - ey) * r / q + ey) / self.minor))
            t = math.hypot(tx, ty)
            tx /= t 
            ty /= t 

        #  best guesstimate
        xc = self.major * tx
        yc = self.minor * ty

        # return the estimated distance
        return math.sqrt( (xc - x0) ** 2.0 + (yc - y0) ** 2.0 )

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
