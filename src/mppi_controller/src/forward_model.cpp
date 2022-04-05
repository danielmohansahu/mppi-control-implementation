#include <mppi_controller/forward_model.h>

namespace mppi
{

Eigen::MatrixXf ForwardModel::rollout(const Eigen::Ref<Statef> state,
                                      const Eigen::Ref<Matrix> commands)
{
  // @TODO how to handle time delays?


  // sanity check input dimensions
  assert(commands.rows() % CONTROL_DIM == 0, "'commands' must be a sequence of controls")

  const size_t steps = commands.rows() / CONTROL_DIM;
  const size_t rollouts = commands.cols();

  // construct mapping matrix from commanded wheel velocities -> velocities
  //  https://www.joydeepb.com/Publications/icra2019_skid_steer.pdf
  //  Equation #8
  Matrix velocity_map = Matrix::Zeros(VEL_DIM, CONTROL_DIM);
  velocity_map << (wheel_separation * (1 - slip_left) / 2), (wheel_separation * (1 - slip_right) / 2),
                  (icr * (1 - slip_left)), (icr * (1 - slip_right)),
                  (slip_left - 1), (1 - slip_left);
  velocity_map *= wheel_radius / wheel_separation;

  // initialize results; it's 1 longer than the command sequence to account for the initial state
  Matrix trajectories = Eigen::Zeros(POS_DIM * (steps + 1), rollouts);
  for (auto j = 0; j != rollouts; ++j)
    trajectories.block(0, j, STATE_DIM, 1) = state;

  // step through each sequence of commands
  for (auto j = 0; j != rollouts; ++j)
    // then, step through each timestep in this sequence
    for (auto i = 1; i != steps + 1; ++i)
    {
      // get this particular command
      const auto&& command = commands.block(CONTROL_DIM * i, j, CONTROL_DIM, 1);

      // calculate velocities from forward model
      auto&& velocities = trajectories.block(STATE_DIM * i + VEL_DIM, j, VEL_DIM, 1)
      velocities = velocity_map * command;
      
      // integrate velocities to get positions
      const auto&& prev_pos = trajectories.block(STATE_DIM * (i - 1), j, POS_DIM, 1);
      trajectories.block(STATE_DIM * i, j, POS_DIM, 1) = prev_pos + velocities * dt;
    }

  // return expected trajectories
  return trajectories;
}

} // namespace mppi
