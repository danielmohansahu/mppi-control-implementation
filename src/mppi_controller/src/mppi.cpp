/* mppi.cpp
 *
 * Implementation of core MPPI class
 */

#include <mppi_controller/mppi.h>

namespace mppi
{

MPPI::MPPI(const std::shared_ptr<ForwardModel> model)
 : model_(model), options_(new Options())
{
}

MPPI::MPPI(const std::shared_ptr<ForwardModel> model, const std::shared_ptr<Options> options)
 : model_(model), options_(options)
{
}

void MPPI::setGoal(const geometry_msgs::PoseStamped& goal)
{
  ROS_INFO_NAMED("MPPI", "Received new goal.");

  // sanity checks
  assert(goal.header.frame_id == options_->frame_id && "Received unexpected frame ID for goal!");

  // construct new goal
  Posef new_goal = Posef::Zero();
  new_goal(0,0) = goal.pose.position.x;
  new_goal(1,0) = goal.pose.position.y;
  
  // @TODO include heading!

  goal_ = new_goal;
}

void MPPI::clear()
{
  ROS_INFO_NAMED("MPPI", "Stopping planning.");
  goal_ = std::nullopt;
}

geometry_msgs::Twist MPPI::plan([[maybe_unused]] const geometry_msgs::PoseStamped& pose)
{
  ROS_INFO_NAMED("MPPI", "'plan' not implemented!!!");
  return geometry_msgs::Twist();
}

Matrix MPPI::sample(const Eigen::Ref<Posef> pose)
{
  // stochastically generate a set of potential trajectories from the given pose


}

float MPPI::cost(const Eigen::Ref<Matrix> trajectory)
{
  // return the cost of the given trajectory

  // sanity checks
  assert(goal_ && "Can't determine the cost of a given trajectory without a goal!");
  assert(trajectory.cols() == 1 && "Trajectory needs to be a vector, not matrix.");
  assert(trajectory.rows() % STATE_DIM == 0 && "Trajectory matrix has invalid dimensions.");
  const size_t steps = (trajectory.rows() / STATE_DIM);

  // minimum viable example: punish deviance from desired position and velocity
  float cumulative = 0;
  for (size_t i = 0; i != steps; ++i)
  {
    // get expected state variables
    const auto&& xy = trajectory.block(STATE_DIM * i, 0, 2, 1);
    const auto&& dx = trajectory(STATE_DIM * i + 3, 0);

    // add deviance from desired goal
    cumulative += options_->weight_dist * (goal_->block(0,0,2,1) - xy).norm();

    // add deviance from desired velocity
    cumulative += options_->weight_vel * std::abs(dx - options_->desired_vel);
  }

  return cumulative;
}

Matrix MPPI::evaluate(const Eigen::Ref<Posef> pose)
{
  // determine the next best trajectory from the given pose

  // generate potential command swaths
  Matrix potentials = sample(pose);

  // find the lowest cost trajectory of the bunch
  Matrix best_trajectory;
  float best_cost = std::numeric_limits<float>::max();
  for (auto j = 0; j != potentials.cols(); ++j)
    if (float c = cost(potentials.col(j)); c < best_cost)
    {
      best_cost = c;
      best_trajectory = potentials.col(j);
    }
  
  // sanity check that we got something
  assert(best_cost < std::numeric_limits<float>::max() && "Failed to get a valid plan.");

  ROS_DEBUG("Found next trajectory.");
  return best_trajectory;
}

} // namespace mppi
