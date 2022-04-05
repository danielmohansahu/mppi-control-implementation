/* visualization.h
 *
 * Tools for publishing and visualizing.
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

// custom
#include "types.h"

namespace mppi
{

// publish the given set of trajectories
void publish(const ros::Publisher& pub,
             const Eigen::Ref<Matrix> trajectories,
             const std::string& frame_id,
             const ros::Time& start_time);

} // namespace mppi
