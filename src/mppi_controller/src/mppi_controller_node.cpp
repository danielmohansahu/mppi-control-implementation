/* mppi_controller_node.cpp
 *
 * ROS interface for MPPI controller
 */

// ROS
#include <ros/ros.h>

// custom
#include <mppi_controller/forward_model.h>
#include <mppi_controller/mppi.h>

int main(int argc, char** argv)
{
  // start ROS node
  ros::init(argc, argv, "mppi_controller_node");
  ros::NodeHandle nh;

  // construct forward model
  auto forward_model = std::make_shared<mppi::ForwardModel>();
  ROS_INFO_NAMED("mppi_controller_node", "ForwardModel instantiated.");

  // construct base controller
  mppi::MPPI controller(forward_model, nh);
  ROS_INFO_NAMED("mppi_controller_node", "MPPI Controller instantiated.");

  // spin until shutdown
  ros::spin();

  return 0;
};
