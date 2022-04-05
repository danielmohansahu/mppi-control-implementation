/* mppi.cpp
 *
 * Implementation of core MPPI class
 */

#include <mppi_controller/mppi.h>

namespace mppi
{

MPPI::MPPI(const std::shared_ptr<ForwardModel> model, const Options& options)
 : model_(model), options_(options)
{
}

void MPPI::setGoal(const geometry_msgs::PoseStamped goal)
{
  ROS_INFO_NAMED("MPPI", "Received new goal.");
  goal_ = goal;
}

void MPPI::clear()
{
  ROS_INFO_NAMED("MPPI", "Stopping planning.");
  goal_ = std::nullopt;
}

geometry_msgs::Twist MPPI::plan(const geometry_msgs::PoseStamped pose)
{
  ROS_INFO_NAMED("MPPI", "'plan' not implemented!!!");
  return geometry_msgs::Twist();
}


} // namespace mppi
