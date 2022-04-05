/* forward_model.h
 *
 * Forward Model of the platform we expect to control.
 */

#pragma once

// STL
#include <atomic>

// Eigen
#include <eigen3/Eigen/Core>

// custom
#include "types.h"

namespace mppi
{

/* skid steer platform friction model
 * 
 * https://www.joydeepb.com/Publications/icra2019_skid_steer.pdf
 */
struct ForwardModel
{
  // nominal Jackal physical parameters
  std::atomic<float> wheel_radius {0.098};
  std::atomic<float> wheel_seperation {0.262};

  // time-dependent coefficients
  //  defaults result in a simple friction-free kinematic model
  std::atomic<float> icr {1.0};
  std::atomic<float> slip_left {0.0};
  std::atomic<float> slip_right {0.0};

  // simulate control delay
  std::atomic<float> delay {0.0};

  // simulation specific parameters
  std::atomic<float> dt {0.02};

  // predict the output of the given control sequence
  Eigen::MatrixXf rollout(const Eigen::Ref<Statef> state, const Eigen::Ref<Matrix> commands) const;
};

} // namespace mppi
