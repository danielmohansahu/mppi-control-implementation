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
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// custom
#include "types.h"
#include "forward_model.h"
#include "visualization.h"

namespace mppi
{

// convenience method to get Yaw from Quaternion
// https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
static inline float yaw_from_quat(const geometry_msgs::Quaternion& q)
{
  return std::atan2(2.0 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
}

// controller parameters
struct Options
{
  // general parameters
  std::string frame_id {"jackal/odom"}; // planning frame
  float horizon {5.0};                  // planning horizon, seconds
  float dt {0.02};                      // timestep between planning iterations
  float goal_radius {1.0};              // radius around goal to consider "achieved"

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
  MPPI(const std::shared_ptr<ForwardModel> model, ros::NodeHandle& nh);
  MPPI(const std::shared_ptr<ForwardModel> model, ros::NodeHandle& nh, const std::shared_ptr<Options> options);

  // set target goal; this clears extant plans and stops planning
  void setGoal(const geometry_msgs::PoseStamped& goal);

  // clear current goal and stop planning
  void clear();

  // plan from the given pose, i.e. the current position
  //  we expect this to be called at the control loop rate, e.g. 50Hz
  geometry_msgs::Twist plan(const nav_msgs::Odometry& state);

 private:

  // generate a set of potential trajectories from the given pose
  Matrix sample() const;

  // evaluate the cost of the given trajectory
  float cost(const Eigen::Ref<Matrix> trajectory) const;

  // select the best next trajectory
  Matrix evaluate(const Eigen::Ref<Statef> state) const;

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
  mutable std::default_random_engine random_number_generator_;
  mutable std::normal_distribution<float> random_distribution_;

  // debug publisher to visualize trajectories
  ros::Publisher debug_pub_;
};

} // namespace mppi
