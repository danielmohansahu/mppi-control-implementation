/* mppi.cpp
 *
 * Implementation of core MPPI class
 */

#include <mppi_controller/mppi.h>

namespace mppi
{

MPPI::MPPI(const std::shared_ptr<ForwardModel> model, ros::NodeHandle& nh)
 : model_(model), options_(new Options()), random_distribution_(0.0, options_->std)
{
  debug_pub_ = nh.advertise<visualization_msgs::MarkerArray>("rollouts", 1);
}

MPPI::MPPI(const std::shared_ptr<ForwardModel> model, ros::NodeHandle& nh, const std::shared_ptr<Options> options)
 : model_(model), options_(options), random_distribution_(0.0, options_->std)
{
  debug_pub_ = nh.advertise<visualization_msgs::MarkerArray>("rollouts", 1);
}

void MPPI::setGoal(const geometry_msgs::PoseStamped& goal)
{
  ROS_INFO_NAMED("MPPI", "Received new goal.");

  // sanity checks
  assert(goal.header.frame_id == options_->frame_id && "Received unexpected frame ID for goal!");

  // clear any existing goals
  clear();

  // construct new goal
  Posef new_goal = Posef::Zero();
  new_goal(0,0) = goal.pose.position.x;
  new_goal(1,0) = goal.pose.position.y;
  new_goal(2,0) = yaw_from_quat(goal.pose.orientation);
  goal_ = new_goal;

  // initialize control sequence
  const size_t steps = std::round(options_->horizon / options_->dt);
  optimal_control_ = Matrix::Zero(steps * CONTROL_DIM, 1);
}

void MPPI::clear()
{
  ROS_INFO_NAMED("MPPI", "Clearing any extant plans.");
  goal_ = std::nullopt;
  optimal_control_ = std::nullopt;
}

geometry_msgs::Twist MPPI::plan(const nav_msgs::Odometry& state)
{
  // sanity checks
  assert(state.header.frame_id == options_->frame_id && "Received unexpected frame_id in state.");

  // if we don't have a goal command 0 velocity
  if (!goal_)
    return geometry_msgs::Twist();

  // sanity check planning rate, and warn if we're going unexpectedly fast / slow
  static ros::Time last_plan_call = ros::Time::now();
  if (const auto delta = ros::Time::now() - last_plan_call; delta > ros::Duration(options_->dt * 1.1))
    ROS_WARN_THROTTLE_NAMED(5, "MPPI", "Planning rate more than 10%% slower than expected.");
  else if (delta < ros::Duration(options_->dt * 0.9))
    ROS_WARN_THROTTLE_NAMED(5, "MPPI", "Planning rate more than 10%% faster than expected.");

  // convert given ROS state into Eigen
  Statef eigen_state;
  eigen_state << state.pose.pose.position.x,
                 state.pose.pose.position.y,
                 yaw_from_quat(state.pose.pose.orientation),
                 state.twist.twist.linear.x,
                 state.twist.twist.linear.y,
                 state.twist.twist.angular.z;

  // plan and update 
  optimal_control_ = evaluate(eigen_state);

  // convert latest command into a twist
  const Controlf first_cmd = optimal_control_->block(0,0,CONTROL_DIM,1);
  return model_->toMessage(first_cmd);
}

Matrix MPPI::sample() const
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

float MPPI::cost(const Eigen::Ref<Matrix> trajectory) const
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

Matrix MPPI::evaluate(const Eigen::Ref<Statef> state) const
{
  // determine the next best trajectory from the given pose

  // generate potential command swaths
  Matrix commands = sample();

  // get expected state of each command
  Matrix potentials = model_->rollout(state, commands);

  // track costs of each trajectory, for debugging
  std::vector<float> costs;
  costs.reserve(potentials.cols());

  // find the lowest cost trajectory of the bunch
  float best_cost = std::numeric_limits<float>::max();
  size_t index = std::numeric_limits<size_t>::max();
  for (auto j = 0; j != potentials.cols(); ++j)
  {
    float c = cost(potentials.col(j));
    if (c < best_cost)
    {
      best_cost = c;
      index = j;
    }
    costs.push_back(c);
  }
  
  // sanity check that we got something
  assert(best_cost < std::numeric_limits<float>::max() && "Failed to get a valid plan.");

  ROS_DEBUG("Found next trajectory.");
  Matrix command = commands.col(index);

  // publish debugging information
  visualize(debug_pub_, potentials, costs, index, options_->frame_id, ros::Time::now());

  return command;
}

} // namespace mppi
