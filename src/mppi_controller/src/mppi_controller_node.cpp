/* mppi_controller_node.cpp
 *
 * Top level node executable for MPPI Server and Controller
 */

// ROS
#include <ros/ros.h>

// custom
#include <mppi_controller/mppi_server.h>

int main(int argc, char** argv)
{
  // start ROS node
  ros::init(argc, argv, "mppi_controller_node");
  ros::NodeHandle nh;

  // allow asynchronous callback execution
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // create server node
  mppi::MPPIServer server {nh};

  // wait until shutdown
  ros::waitForShutdown();

  return 0;
}
