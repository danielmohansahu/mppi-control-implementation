/* mppi.cpp
 *
 * Implementation of core MPPI class
 */

#include <mppi_controller/mppi.h>

namespace mppi
{

MPPI::MPPI(const std::shared_ptr<ForwardModel> model)
 : model_(model), options_(new Options()), random_distribution_(0.0, options_->std)
{
}

MPPI::MPPI(const std::shared_ptr<ForwardModel> model, const std::shared_ptr<Options> options)
 : model_(model), options_(options), random_distribution_(0.0, options_->std)
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

geometry_msgs::Twist MPPI::plan([[maybe_unused]] const nav_msgs::Odometry& state)
{
  ROS_INFO_NAMED("MPPI", "'plan' not implemented!!!");
  return geometry_msgs::Twist();
}

Matrix MPPI::sample()
{
  // stochastically generate a set of potential trajectories from the given pose
  const size_t steps = std::round(options_->horizon / options_->dt);

  // sanity checks
  assert(optimal_control_ && "No existing optimal_control_ sequence!");
  assert(static_cast<size_t>(optimal_control_->rows()) == steps * CONTROL_DIM
         && "Mismatch with expected control dimensions.");

  // initialize results from existing control distribution
  Matrix commands = Matrix::Zero(steps * CONTROL_DIM, options_->rollouts);
  for (auto j = 0; j != commands.cols(); ++j)
    commands.col(j) = *optimal_control_;

  // perturb the distributions
  // @TODO review the paper and do this more methodologically
  commands.unaryExpr(
    [this] (float x) -> float { return x + random_distribution_(random_number_generator_); });

  return commands;
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
    const auto dx = trajectory(STATE_DIM * i + 3, 0);

    // add deviance from desired goal
    cumulative += options_->weight_dist * (goal_->block(0,0,2,1) - xy).norm();

    // add deviance from desired velocity
    cumulative += options_->weight_vel * std::abs(dx - options_->desired_vel);
  }

  return cumulative;
}

std::tuple<Matrix,Matrix> MPPI::evaluate(const Eigen::Ref<Statef> state)
{
  // determine the next best trajectory from the given pose

  // generate potential command swaths
  Matrix commands = sample();

  // get expected state of each command
  Matrix potentials = model_->rollout(state, commands);

  // find the lowest cost trajectory of the bunch
  float best_cost = std::numeric_limits<float>::max();
  size_t index = std::numeric_limits<size_t>::max();
  for (auto j = 0; j != potentials.cols(); ++j)
    if (float c = cost(potentials.col(j)); c < best_cost)
    {
      best_cost = c;
      index = j;
    }
  
  // sanity check that we got something
  assert(best_cost < std::numeric_limits<float>::max() && "Failed to get a valid plan.");

  ROS_DEBUG("Found next trajectory.");
  Matrix command = commands.col(index);
  Matrix trajectory = potentials.col(index);
  return {command, trajectory};
}

} // namespace mppi
