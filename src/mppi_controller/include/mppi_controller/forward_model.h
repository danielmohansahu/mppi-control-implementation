/* forward_model.h
 *
 * Forward Model of the platform we expect to control.
 */

#pragma once

// STL
#include <atomic>

// ROS
#include <geometry_msgs/Twist.h>
#include <mppi_controller/MPPIOptionsConfig.h>

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
  // convenience typedef
  using Options = mppi_controller::MPPIOptionsConfig;

  // predict the output of the given control sequence
  Eigen::MatrixXf rollout(const Eigen::Ref<Statef> state, const Eigen::Ref<Matrix> commands) const;

  // convert target wheel angular velocities into a twist message
  geometry_msgs::Twist toMessage(const Controlf& cmd) const;

  // apply control constraints to the given command set
  void constrain(Eigen::Ref<Matrix> commands) const;

  // required constructor
  ForwardModel() = delete;
  explicit ForwardModel(const std::shared_ptr<Options>& opts)
    : opts_(opts)
  {}

 private:
  // get the matrix that maps angular velocities to twist
  VelMatrix get_velocity_map() const;

  // configurable parameters
  const std::shared_ptr<Options> opts_;

};

} // namespace mppi
