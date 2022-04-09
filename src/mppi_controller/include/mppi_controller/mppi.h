/* mppi.h
 *
 * Core Model Predictive Path Integral controller class
 */

#pragma once

// STL
#include <tuple>
#include <optional>
#include <variant>
#include <memory>
#include <limits>
#include <random>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <mppi_controller/MPPIOptionsConfig.h>
#include <mppi_controller/WaypointGoal.h>
#include <mppi_controller/FollowCourseGoal.h>

// custom
#include "types.h"
#include "forward_model.h"
#include "visualization.h"

namespace mppi
{

// convenience method to get Yaw from Quaternion
// https://github.com/ros/geometry2/blob/noetic-devel/tf2/include/tf2/impl/utils.h
static inline float yaw_from_quat(const geometry_msgs::Quaternion& q)
{
  const float sqx = q.x * q.x;
  const float sqy = q.y * q.y;
  const float sqz = q.z * q.z;
  const float sqw = q.w * q.w;

  // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
  if (const float sarg = -2 * (q.x * q.z - q.w * q.y) / (sqx + sqy + sqz + sqw); sarg <= -0.99999)
    return -2.0 * atan2(q.y, q.x);
  else if (sarg >= 0.99999)
    return 2.0 * atan2(q.y, q.x);
  else
    return std::atan2(2.0 * (q.x * q.y + q.w * q.z), sqw + sqx - sqy - sqz);
}

// core MPPI algorithm implementation class
class MPPI
{
 public:
  using Options           = mppi_controller::MPPIOptionsConfig;
  using WaypointGoal      = mppi_controller::WaypointGoal;
  using FollowCourseGoal  = mppi_controller::FollowCourseGoal;

  // construct new MPPI instance
  MPPI(const std::shared_ptr<ForwardModel> model, ros::NodeHandle& nh, const std::shared_ptr<Options> options);

  // set target goal; this clears extant plans and stops planning
  void setGoal(const WaypointGoal& goal);

  // overload for course following goals
  void setGoal(const FollowCourseGoal& goal);

  // clear current goal and stop planning
  void clear();

  // plan from the given pose, i.e. the current position
  //  we expect this to be called at the control loop rate, e.g. 50Hz
  geometry_msgs::Twist plan(const nav_msgs::Odometry& state);

  // enumeration of supported goal types
  enum GoalTypes { UNSET, WAYPOINT, FOLLOWCOURSE };

 private:

  // generate a set of potential trajectories from the given pose
  Matrix sample() const;

  // evaluate the cost of the given trajectory
  float cost(const Eigen::Ref<Matrix> trajectory) const;

  // select the best next trajectory
  Matrix evaluate(const Eigen::Ref<Statef> state, const nav_msgs::Odometry& pose) const;

  // forward model
  const std::shared_ptr<ForwardModel> model_;

  // parameters
  const std::shared_ptr<Options> options_;

  // current goal variables
  GoalTypes goal_type_;
  std::variant<std::monostate, WaypointGoal, FollowCourseGoal> goal_;

  // current control sequence and corresponding expected trajectory
  std::optional<Matrix> optimal_control_;

  // random number generation
  mutable std::mt19937 random_generator_;
  mutable std::normal_distribution<float> random_distribution_;

  // debug publisher to visualize trajectories
  ros::Publisher debug_pub_;

  // timing analysis
  std::optional<ros::Time> last_plan_call_;
};

} // namespace mppi
