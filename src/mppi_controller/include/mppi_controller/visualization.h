/* visualization.h
 *
 * Tools for publishing and visualizing.
 */

#pragma once

// STL
#include <algorithm>

// ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

// custom
#include "types.h"

namespace mppi
{

// publish the given set of trajectories
void visualize(const ros::Publisher& pub,
               const Eigen::Ref<Matrix> trajectories,
               const std::vector<float>& costs,
               const size_t winner_idx,
               const std::string& frame_id,
               const ros::Time& start_time);

} // namespace mppi
