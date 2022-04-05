/* visualization.cpp
 *
 * Tools for publishing and visualizing.
 */

// custom
#include <mppi_controller/visualization.h>

namespace mppi
{

// publish the given set of trajectories
void publish(const ros::Publisher& pub,
             const Eigen::Ref<Matrix> trajectories,
             const std::string& frame_id,
             const ros::Time& start_time)
{
  // return early if nobody is listening
  if (pub.getNumSubscribers() == 0)
    return;

  // sanity check given trajectory
  assert(trajectories.rows() % STATE_DIM == 0);
  const size_t steps = trajectories.rows() / STATE_DIM;

  // initialize visualization message
  visualization_msgs::MarkerArray msg;

  // add a new marker for each trajectory
  for (auto j = 0; j != trajectories.cols(); ++j)
  {
    // initialize new marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = start_time;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.25);
    marker.frame_locked = true;
    marker.points.reserve(steps);

    // add a new point for each step in the trajectory
    for (size_t i = 0; i != steps; ++i)
    {
      geometry_msgs::Point point;
      point.x = trajectories(STATE_DIM * i, j);
      point.x = trajectories(STATE_DIM * i + 1, j);
      marker.points.push_back(point);
    }

    // push this back to our marker array
    msg.markers.push_back(marker);
  }

  // publish!
  pub.publish(msg);
}

} // namespace mppi
