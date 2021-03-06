#include <mppi_controller/forward_model.h>

namespace mppi
{

geometry_msgs::Twist ForwardModel::toMessage(const Controlf& cmd) const
{
  // get expected velocities from this command
  Velf vels = get_velocity_map() * cmd;

  // convert wheel velocities into Twist commands
  geometry_msgs::Twist msg;
  msg.linear.x = vels(0,0);
  msg.angular.z = vels(2,0);
  return msg;
}

void ForwardModel::constrain(Eigen::Ref<Matrix> commands) const
{
  // apply control constraints to the desired commands

  // clamp to maximum allowed velocity constraints
  commands = commands.cwiseMin(opts_->max_omega).cwiseMax(opts_->min_omega);

  // iterate through time steps, applying angular acceleration constraints
  //  @TODO figure out how to do vectorize this operation
  for (auto j = 0; j != commands.cols(); ++j)
    for (auto i = 1; i != commands.rows(); ++i)
    {
      const auto alpha = (commands(i,j) - commands(i-1,j)) / opts_->dt;
      if (alpha > opts_->max_alpha)
        commands(i,j) = commands(i-1,j) + opts_->max_alpha * opts_->dt;
      else if (alpha < opts_->min_alpha)
        commands(i,j) = commands(i-1,j) + opts_->min_alpha * opts_->dt;
    }
}

VelMatrix ForwardModel::get_velocity_map() const
{
  // construct mapping matrix from commanded angular wheel velocities -> Twist velocities

  //  https://www.joydeepb.com/Publications/icra2019_skid_steer.pdf
  //  Equation #8
  VelMatrix velocity_map = VelMatrix::Zero();
  velocity_map << (opts_->wheel_separation * (1 - opts_->slip_left) / 2), (opts_->wheel_separation * (1 - opts_->slip_right) / 2),
                  (opts_->icr * (1 - opts_->slip_left)), (opts_->icr * (opts_->slip_right - 1)),
                  (opts_->slip_left - 1), (1 - opts_->slip_right);
  velocity_map *= opts_->wheel_radius / opts_->wheel_separation;
  return velocity_map;
}

Eigen::MatrixXf ForwardModel::rollout(const Eigen::Ref<Statef> state,
                                      const Eigen::Ref<Matrix> commands) const
{
  // @TODO how to handle time delays?

  // sanity check input dimensions
  assert(commands.rows() % CONTROL_DIM == 0 && "'commands' must be a sequence of controls");

  const size_t steps = commands.rows() / CONTROL_DIM;
  const size_t rollouts = commands.cols();

  // construct mapping matrix from commanded angular wheel velocities -> Twist velocities
  VelMatrix velocity_map = get_velocity_map();

  // initialize results; it's 1 longer than the command sequence to account for the initial state
  Matrix trajectories = Matrix::Zero(STATE_DIM * (steps + 1), rollouts);
  for (size_t j = 0; j != rollouts; ++j)
    trajectories.block(0, j, STATE_DIM, 1) = state;

  // step through each sequence of commands
  for (size_t j = 0; j != rollouts; ++j)
    // then, step through each timestep in this sequence
    for (size_t i = 0; i != steps; ++i)
    {
      // get this particular command
      const auto&& command = commands.block(CONTROL_DIM * i, j, CONTROL_DIM, 1);

      // calculate velocities from forward model
      auto&& vel = trajectories.block(STATE_DIM * (i + 1) + POS_DIM, j, VEL_DIM, 1);
      vel = velocity_map * command;
      
      // integrate velocities to get positions
      const auto&& pos = trajectories.block(STATE_DIM * i, j, POS_DIM, 1);  // previous step's position
      trajectories(STATE_DIM * (i + 1), j) =
        pos(0,0) + vel(0,0) * std::cos(pos(2,0)) * opts_->dt + vel(1,0) * std::sin(pos(2,0)) * opts_->dt;
      trajectories(STATE_DIM * (i + 1) + 1, j) =
        pos(1,0) + vel(1,0) * std::cos(pos(2,0)) * opts_->dt + vel(0,0) * std::sin(pos(2,0)) * opts_->dt;
      trajectories(STATE_DIM * (i + 1) + 2, j) = pos(2,0) + vel(2,0) * opts_->dt;
    }

  // return expected trajectories
  return trajectories;
}

} // namespace mppi
