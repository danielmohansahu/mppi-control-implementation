/* visualization.cpp
 *
 * Tools for publishing and visualizing.
 */

// custom
#include <mppi_controller/visualization.h>

namespace mppi
{

// publish the given set of trajectories
void visualize(const ros::Publisher& pub,
               const Eigen::Ref<Matrix> trajectories,
               const std::vector<float>& costs,
               const size_t winner_idx,
               const std::string& frame_id,
               const ros::Time& start_time)
{
  // return early if nobody is listening
  if (pub.getNumSubscribers() == 0)
    return;

  // sanity checks
  assert(static_cast<size_t>(trajectories.cols()) == costs.size());
  assert(trajectories.rows() % STATE_DIM == 0);
  const size_t steps = trajectories.rows() / STATE_DIM;

  // get min, max costs to scale colors
  const float min_cost = costs[winner_idx]; // better be!
  const float max_cost = *std::max_element(costs.cbegin(), costs.cend());
  auto color_scale = [min_cost, max_cost] (float cost) -> std_msgs::ColorRGBA
  {
    // normalize cost
    float norm = (cost - min_cost) / (max_cost - min_cost);
    // scale red and green; red == high cost, green == low cost
    std_msgs::ColorRGBA msg;
    msg.r = norm;
    msg.g = 1 - norm;
    return msg;
  };

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

    // assign color based on cost
    marker.color = color_scale(costs[j]);

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
