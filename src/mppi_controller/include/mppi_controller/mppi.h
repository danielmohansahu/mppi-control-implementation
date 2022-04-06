/* mppi.h
 *
 * Core Model Predictive Path Integral controller class
 */

#pragma once

// STL
#include <tuple>
#include <optional>
#include <memory>
#include <limits>
#include <random>

// ROS
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// custom
#include "types.h"
#include "forward_model.h"

namespace mppi
{

// controller parameters
struct Options
{
  // general parameters
  std::string frame_id {"jackal/odom"}; // planning frame
  float horizon {5.0};                  // planning horizon, seconds
  float dt {0.02};                      // timestep between planning iterations

  // sampling parameters
  float std {0.25};             // standard deviation of sampling distribution
  unsigned int rollouts {100};  // number of rollouts to evaluate

  // cost parameters
  float weight_dist {1.0};  // penalize euclidean distance
  float weight_vel {1.0};   // penalize deviance from desired velocity
  float desired_vel {2.0};  // desired velocity (m/s)
};

class MPPI
{
 public:

  // construct new MPPI instance
  MPPI(const std::shared_ptr<ForwardModel> model);
  MPPI(const std::shared_ptr<ForwardModel> model, const std::shared_ptr<Options> options);

  // set target goal; this clears extant plans and stops planning
  void setGoal(const geometry_msgs::PoseStamped& goal);

  // clear current goal and stop planning
  void clear();

  // plan from the given pose, i.e. the current position
  //  we expect this to be called at the control loop rate, e.g. 50Hz
  geometry_msgs::Twist plan(const nav_msgs::Odometry& state);

 private:

  // generate a set of potential trajectories from the given pose
  Matrix sample();

  // evaluate the cost of the given trajectory
  float cost(const Eigen::Ref<Matrix> trajectory);

  // select the best next trajectory
  std::tuple<Matrix,Matrix> evaluate(const Eigen::Ref<Statef> state);

  // forward model
  const std::shared_ptr<ForwardModel> model_;

  // parameters
  const std::shared_ptr<Options> options_;

  // current goal
  std::optional<Posef> goal_;

  // current control sequence and corresponding expected trajectory
  std::optional<Matrix> optimal_control_;
  std::optional<Matrix> optimal_trajectory_;

  // random number generation
  std::default_random_engine random_number_generator_;
  std::normal_distribution<float> random_distribution_;
};

} // namespace mppi
