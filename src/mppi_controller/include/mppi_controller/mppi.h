/* mppi.h
 *
 * Core Model Predictive Path Integral controller class
 */

#pragma once

// STL
#include <optional>
#include <memory>

// ROS
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

// custom
#include "types.h"
#include "forward_model.h"

namespace mppi
{

// controller parameters
struct Options
{
};

class MPPI
{
 public:

  // construct new MPPI instance
  MPPI(const std::shared_ptr<ForwardModel> model, const Options& options = Options());

  // set target goal; this clears extant plans and stops planning
  void setGoal(const geometry_msgs::PoseStamped goal);

  // clear current goal and stop planning
  void clear();

  // plan from the given pose, i.e. the current position
  geometry_msgs::Twist plan(const geometry_msgs::PoseStamped pose);

 private:

  // forward model
  const std::shared_ptr<ForwardModel> model_;

  // parameters
  const Options options_;

  // current goal
  std::optional<geometry_msgs::PoseStamped> goal_;
};

} // namespace mppi
