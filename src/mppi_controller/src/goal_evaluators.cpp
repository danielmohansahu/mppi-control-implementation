/* goal_evaluators.cpp
 *
 * Implementation of classes for goal evaluation (costing).
 */

#include <mppi_controller/goal_evaluators.h>

namespace mppi
{

std::tuple<size_t, std::vector<float>>
WaypointEvaluator::cost(const Eigen::Ref<Matrix> trajectories) const
{
  // return the best (lowest cost) of the given trajectories

  // sanity checks
  assert(trajectories.rows() % STATE_DIM == 0 && "Trajectories matrix has invalid dimensions.");
  const size_t steps = (trajectories.rows() / STATE_DIM);

  // initialization
  float best_cost = std::numeric_limits<float>::max();
  size_t best_indx = 0;
  const float x_des = goal_.pose.pose.position.x;
  const float y_des = goal_.pose.pose.position.y;
  std::vector<float> costs;
  costs.reserve(steps);

  // iterate through trajectories
  for (auto j = 0; j != trajectories.cols(); ++j)
  {
    // initialize variables for this trajectory
    auto&& trajectory = trajectories.col(j);
    float cost = 0.0;

    // step through this trajectory, accumulating costs
    for (size_t i = 0; i != steps; ++i)
    {
      // get expected state variables for this trajectory step
      const auto x = trajectory(STATE_DIM * i + X_IDX, 0);
      const auto y = trajectory(STATE_DIM * i + Y_IDX, 0);

      // add cost for euclidean distance
      cost += opts_->weight_dist * std::sqrt( std::pow(x_des - x, 2.0) + std::pow(y_des - y, 2.0) );

      // add cost for velocity deviation
      cost += opts_->weight_vel * std::abs(goal_.velocity - trajectory(STATE_DIM * i + DX_IDX, 0));

      // add cost for angular velocity
      cost += opts_->weight_omega * std::abs(trajectory(STATE_DIM * i + DTH_IDX, 0));
    }

    // scale cost to number of points (not really necessary...)
    cost /= steps;
    costs.push_back(cost);

    // keep track of best cost so far
    if (cost < best_cost)
    {
      best_cost = cost;
      best_indx = j;
    }
  }

  // return the trajectory with the best cost and debugging info
  return {best_indx, costs};
}

std::tuple<size_t, std::vector<float>>
FollowCourseEvaluator::cost(const Eigen::Ref<Matrix> trajectories) const
{
  // return the best (lowest cost) of the given trajectories

  // sanity checks
  assert(trajectories.rows() % STATE_DIM == 0 && "Trajectories matrix has invalid dimensions.");
  const size_t steps = (trajectories.rows() / STATE_DIM);

  // initialization
  float best_cost = std::numeric_limits<float>::max();
  size_t best_indx = 0;
  std::vector<float> costs;
  costs.reserve(steps);

  // iterate through trajectories
  for (auto j = 0; j != trajectories.cols(); ++j)
  {
    // initialize variables for this trajectory
    auto&& trajectory = trajectories.col(j);
    float cost = 0.0;

    // step through this trajectory, accumulating costs
    for (size_t i = 0; i != steps; ++i)
    {
      // add cost for euclidean distance
      cost += opts_->weight_dist * ellipse_.dist(trajectory(STATE_DIM * i + X_IDX, 0), trajectory(STATE_DIM * i + Y_IDX, 0));

      // add cost for velocity deviation
      cost += opts_->weight_vel * std::abs(goal_.velocity - trajectory(STATE_DIM * i + DX_IDX, 0));

      // add cost for angular velocity
      cost += opts_->weight_omega * std::abs(trajectory(STATE_DIM * i + DTH_IDX, 0));
    }

    // scale cost to number of points (not really necessary...)
    cost /= steps;
    costs.push_back(cost);

    // keep track of best cost so far
    if (cost < best_cost)
    {
      best_cost = cost;
      best_indx = j;
    }
  }

  // return the trajectory with the best cost and debugging info
  return {best_indx, costs};
}

float FollowCourseEvaluator::Ellipse::dist(float x, float y) const
{
  // @TODO fix this! it's not distance!
  return std::abs(1 - std::pow(x, 2.0) / std::pow(major_, 2.0) - std::pow(y, 2.0) / std::pow(minor_, 2.0));
}



} // namespace mppi
