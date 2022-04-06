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
#include <nav_msgs/Odometry.h>

// custom
#include "types.h"

namespace mppi
{

// publish the given set of trajectories
void visualize(const ros::Publisher& pub,
               const Eigen::Ref<Matrix> trajectories,
               const std::vector<float>& costs,
               const nav_msgs::Odometry& pose,
               const size_t winner_idx);

} // namespace mppi
