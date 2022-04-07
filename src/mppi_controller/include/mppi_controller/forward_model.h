/* forward_model.h
 *
 * Forward Model of the platform we expect to control.
 */

#pragma once

// STL
#include <atomic>

// ROS
#include <geometry_msgs/Twist.h>

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
  std::atomic<float> wheel_separation {0.262};

  // time-dependent coefficients
  //  defaults result in a simple friction-free kinematic model
  std::atomic<float> icr {0.0};
  std::atomic<float> slip_left {0.0};
  std::atomic<float> slip_right {0.0};

  // simulate control delay
  //  @TODO; currently unused
  std::atomic<float> delay {0.0};

  // platform command constraints
  //  calculated from nominal max velocity (2m/s) / wheel radius (~0.1m)
  std::atomic<float> max_omega {2.0};
  std::atomic<float> min_omega {-2.0};

  // simulation specific parameters
  std::atomic<float> dt {0.02};

  // predict the output of the given control sequence
  Eigen::MatrixXf rollout(const Eigen::Ref<Statef> state, const Eigen::Ref<Matrix> commands) const;

  // convert target wheel angular velocities into a twist message
  geometry_msgs::Twist toMessage(const Controlf& cmd) const;

  // apply control constraints to the given command set
  void constrain(Eigen::Ref<Matrix> commands) const;

 private:
  // get the matrix that maps angular velocities to twist
  VelMatrix get_velocity_map() const;
};

} // namespace mppi
