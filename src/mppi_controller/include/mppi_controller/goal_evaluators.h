/* goal_evaluators.h
 *
 * Classes used to evaluate the performance of different
 * types of goals.
 */

#pragma once

// STL
#include <memory>
#include <tuple>
#include <vector>
#include <limits>
#include <math.h>

// Eigen
#include <eigen3/Eigen/Core>

// ROS
#include <mppi_controller/WaypointGoal.h>
#include <mppi_controller/FollowCourseGoal.h>
#include <mppi_controller/MPPIOptionsConfig.h>

// Custom
#include "types.h"

namespace mppi
{

// handle assignment of cost for Waypoint goals
class WaypointEvaluator
{
  // convenience typedefs
  using WaypointGoal = mppi_controller::WaypointGoal;
  using Options = mppi_controller::MPPIOptionsConfig;

 public:
  // expected constructor
  WaypointEvaluator(const WaypointGoal& goal, const std::shared_ptr<Options> opts)
   : opts_(opts), goal_(goal)
  {}
  
  // return best (lowest) cost out of the given trajectories
  // also returns all costs for debugging / visualization
  std::tuple<size_t, std::vector<float>> cost(const Eigen::Ref<Matrix> trajectories) const;

 private:

  // reference to shared options
  const std::shared_ptr<Options> opts_;

  // currently tracked goal
  const WaypointGoal goal_;

};

// handle assignments of cost for FollowCourse goals
class FollowCourseEvaluator
{
  // convenience typedefs
  using FollowCourseGoal = mppi_controller::FollowCourseGoal;
  using Options = mppi_controller::MPPIOptionsConfig;

  // a helper class for evaluating ellipses centered around (0,0)
  class Ellipse
  {
    // ellipse geometry
    float major_, minor_;

   public:
    // constructor
    explicit Ellipse(float major, float minor)
     : major_(major), minor_(minor)
    {}

    // return the euclidean distance to the closest point on the ellipse
    float dist(float x0, float y0) const;
  };

 public:
  // expected constructor
  FollowCourseEvaluator(const FollowCourseGoal& goal, const std::shared_ptr<Options> opts)
   : opts_(opts), goal_(goal), ellipse_(goal.major, goal.minor)
  {}
  
  // return best (lowest) cost out of the given trajectories
  // also returns all costs for debugging / visualization
  std::tuple<size_t, std::vector<float>> cost(const Eigen::Ref<Matrix> trajectories) const;

 private:

  // reference to shared options
  const std::shared_ptr<Options> opts_;

  // currently tracked goal
  const FollowCourseGoal goal_;

  // local ellipse class used in evaluating distance to goal
  const Ellipse ellipse_;
};


} // namespace mppi
