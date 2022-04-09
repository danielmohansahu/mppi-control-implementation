/* mppi.cpp
 *
 * Implementation of core MPPI class
 */

#include <mppi_controller/mppi.h>

namespace mppi
{

MPPI::MPPI(const std::shared_ptr<ForwardModel> model, ros::NodeHandle& nh, const std::shared_ptr<Options> options)
 : model_(model),
   options_(options),
   goal_type_(GoalTypes::UNSET),
   random_generator_(std::random_device{}()),
   random_distribution_(0.0, options_->std)
{
  debug_pub_ = nh.advertise<visualization_msgs::MarkerArray>("rollouts", 1);
}

void MPPI::setGoal(const WaypointGoal& goal)
{
  ROS_INFO_NAMED("MPPI", "Received new goal.");

  // sanity checks
  assert(goal.pose.header.frame_id == options_->frame_id && "Received unexpected frame ID for goal!");

  // clear any existing goals
  clear();

  // construct new goal
  goal_ = goal;
  goal_type_ = GoalTypes::WAYPOINT;
}

void MPPI::setGoal(const FollowCourseGoal& goal)
{
  ROS_INFO_NAMED("MPPI", "Received new goal.");

  // clear any existing goals
  clear();

  // construct new goal
  goal_ = goal;
  goal_type_ = GoalTypes::FOLLOWCOURSE;
}

void MPPI::clear()
{
  ROS_INFO_NAMED("MPPI", "Clearing any extant plans.");

  // set class variables back to defaults
  goal_ = std::monostate();
  goal_type_ = GoalTypes::UNSET;
  optimal_control_ = std::nullopt;
  last_plan_call_ = std::nullopt;

  // initialize control sequence
  const size_t steps = std::round(options_->horizon / options_->dt);
  optimal_control_ = Matrix::Zero(steps * CONTROL_DIM, 1);
}

geometry_msgs::Twist MPPI::plan(const nav_msgs::Odometry& state)
{
  // sanity checks
  assert(state.header.frame_id == options_->frame_id && "Received unexpected frame_id in state.");

  // if we don't have a goal command 0 velocity
  if (goal_type_ == GoalTypes::UNSET)
    return geometry_msgs::Twist();

  // sanity check planning rate, and warn if we're going unexpectedly fast / slow
  static size_t issues = 0;
  if (!last_plan_call_)
    last_plan_call_ = ros::Time::now();
  else
  {
    if (const auto delta = ros::Time::now() - *last_plan_call_; delta > ros::Duration(options_->dt * 1.1))
    {
      ++issues;
      ROS_WARN_STREAM_THROTTLE_NAMED(5, "MPPI", "Planning rate " << delta.toSec() / options_->dt
                                                << " slower than expected. " << issues << " times so far.");
    }
    else if (delta < ros::Duration(options_->dt * 0.9))
    {
      ++issues;
      ROS_WARN_STREAM_THROTTLE_NAMED(5, "MPPI", "Planning rate " << delta.toSec() / options_->dt
                                                << " faster than expected. " << issues << " times so far.");
    }
    // update time for next loop
    last_plan_call_ = ros::Time::now();
  }

  // convert given ROS state into Eigen
  Statef eigen_state;
  eigen_state << state.pose.pose.position.x,
                 state.pose.pose.position.y,
                 yaw_from_quat(state.pose.pose.orientation),
                 state.twist.twist.linear.x,
                 state.twist.twist.linear.y,
                 state.twist.twist.angular.z;

  // plan and update 
  Matrix optimal = evaluate(eigen_state, state);

  // drop the first command, assuming it's been executed, and assign for next iteration
  optimal_control_->block(0, 0, optimal_control_->rows() - CONTROL_DIM, 1) =
    optimal.block(CONTROL_DIM, 0, optimal_control_->rows() - CONTROL_DIM, 1);
  optimal_control_->block(optimal_control_->rows() - CONTROL_DIM - 1, 0, CONTROL_DIM, 1) = 
    Matrix::Zero(CONTROL_DIM, 1);

  // convert latest command into a twist
  const Controlf first_cmd = optimal.block(0,0,CONTROL_DIM,1);
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
  commands += commands.unaryExpr(
    [this] (float) -> float { return random_distribution_(random_generator_); });

  // ensure that one rollout is the same as before, if it were optimal
  commands.col(0) = *optimal_control_;

  return commands;
}

float MPPI::cost(const Eigen::Ref<Matrix> trajectory) const
{
  // return the cost of the given trajectory

  // sanity checks
  assert(goal_type_ != GoalTypes::UNSET && "Can't determine the cost of a given trajectory without a goal!");
  assert(trajectory.cols() == 1 && "Trajectory needs to be a vector, not matrix.");
  assert(trajectory.rows() % STATE_DIM == 0 && "Trajectory matrix has invalid dimensions.");
  const size_t steps = (trajectory.rows() / STATE_DIM);

  // minimum viable example: punish deviance from desired position and velocity
  float cumulative = 0;
  for (size_t i = 0; i != steps; ++i)
  {
    // get expected state variables
    const auto x = trajectory(STATE_DIM * i, 0);
    const auto y = trajectory(STATE_DIM * i + 1, 0);
    const auto dx = trajectory(STATE_DIM * i + 3, 0);
    const auto dtheta = trajectory(STATE_DIM * i + 4, 0);

    // add deviance from desired goal
    // @TODO clean this up w/ std::visit
    float euclidean_dist = 0;
    float desired_vel = 0;
    switch (goal_type_)
    {
      case GoalTypes::WAYPOINT:
      {
        const auto& pos = std::get<WaypointGoal>(goal_).pose.pose.position;
        euclidean_dist = std::sqrt( std::pow(pos.x - x, 2.0) + std::pow(pos.y - y, 2.0) );
        desired_vel = std::get<WaypointGoal>(goal_).velocity;
        break;
      }
      case GoalTypes::FOLLOWCOURSE:
      {
        const auto& g = std::get<FollowCourseGoal>(goal_);
        euclidean_dist = std::abs(1 - std::pow(x, 2.0) / std::pow(g.major, 2.0) - std::pow(y, 2.0) / std::pow(g.minor, 2.0));
        desired_vel = std::get<WaypointGoal>(goal_).velocity;
        break;
      }
      default: throw std::runtime_error("Reached unreachable code.");
    }
    cumulative += options_->weight_dist * euclidean_dist;

    // add deviance from desired velocity
    cumulative += options_->weight_vel * std::abs(dx - desired_vel);

    // penalize rotation
    cumulative += options_->weight_omega * std::abs(dtheta);
  }

  return cumulative;
}

Matrix MPPI::evaluate(const Eigen::Ref<Statef> state, const nav_msgs::Odometry& pose) const
{
  // determine the next best trajectory from the given pose

  // generate potential command swaths
  Matrix commands = sample();

  // constrain commands
  model_->constrain(commands);

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
  visualize(debug_pub_, potentials, costs, pose, index);

  return command;
}

} // namespace mppi
