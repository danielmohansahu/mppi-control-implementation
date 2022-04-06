/* mppi_controller_node.cpp
 *
 * ROS interface for MPPI controller
 */

// STL
#include <mutex>
#include <optional>

// ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

// custom
#include <mppi_controller/forward_model.h>
#include <mppi_controller/mppi.h>

int main(int argc, char** argv)
{
  // mutex to guard concurrent read / write access
  std::mutex mutex;

  // start ROS node
  ros::init(argc, argv, "mppi_controller_node");
  ros::NodeHandle nh;

  // construct forward model
  auto forward_model = std::make_shared<mppi::ForwardModel>();
  ROS_INFO_NAMED("mppi_controller_node", "ForwardModel instantiated.");

  // construct Options struct
  auto options = std::make_shared<mppi::Options>();

  // construct base controller
  mppi::MPPI controller(forward_model, nh, options);
  ROS_INFO_NAMED("mppi_controller_node", "MPPI Controller instantiated.");

  // set up twist publisher to send command velocities to robot
  ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // set up subscriber to capture new goals
  std::optional<geometry_msgs::PoseStamped> current_goal;
  ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "goal",
      1,
      [&current_goal, &controller, &mutex] (const geometry_msgs::PoseStamped::ConstPtr& msg) -> void
      {
        ROS_DEBUG_NAMED("mppi_controller_node", "Updating goal.");
        std::unique_lock<std::mutex> lock(mutex);
        current_goal = *msg;

        // update goal
        controller.setGoal(*msg);
      });

  // set up subscriber to capture robot odometry
  std::optional<nav_msgs::Odometry> current_odometry;
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
      "odom",
      1,
      [&current_odometry, &mutex] (const nav_msgs::Odometry::ConstPtr& msg) -> void
      {
        ROS_DEBUG_NAMED("mppi_controller_node", "Updating odometry.");
        std::unique_lock<std::mutex> lock(mutex);

        // @TODO add odometry noise !
        current_odometry = *msg;
      });

  // allow asynchronous callback execution
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // plan until shutdown
  ROS_INFO_NAMED("mppi_controller_node", "Beginning planning loop...");
  while (ros::ok())
  {
    ros::Time start_time = ros::Time::now();

    // check that we have a goal and odometry
    if (std::unique_lock<std::mutex> lock(mutex); !current_odometry)
      ROS_WARN_THROTTLE_NAMED(5, "mppi_controller_node", "Waiting for initial odometry.");
    else if (!current_goal)
      ROS_INFO_THROTTLE_NAMED(5, "mppi_controller_node", "Waiting for goal.");
    else
    {
      // check if we've achieved our goal
      float dist_squared = std::pow(current_goal->pose.position.x - current_odometry->pose.pose.position.x, 2.0)
                           + std::pow(current_goal->pose.position.y - current_odometry->pose.pose.position.y , 2.0);
      if (dist_squared <= options->goal_radius)
      {
        ROS_INFO_NAMED("mppi_controller_node", "Goal achieved!");
        current_goal = std::nullopt;
        controller.clear();

        // send stop command
        twist_pub.publish(geometry_msgs::Twist());
        continue;
      }

      // if we've made it this far we want to continue planning
      // copy of current odometry
      nav_msgs::Odometry odom_copy = *current_odometry;

      // @TODO add controller noise!
      // @TODO figure out how to release lock early
      twist_pub.publish(controller.plan(odom_copy));
    }

    if (ros::Duration delta = (ros::Time::now() - start_time); delta >= ros::Duration(options->dt))
      // we took too long planning... this is very bad
      ROS_WARN_THROTTLE_NAMED(5, "mppi_controller_node", "Planning takes longer than required loop rate!");
    else
      // sleep for remainder of loop
      (ros::Duration(options->dt) - delta).sleep();
  }

  // shutting down
  return 0;
};
