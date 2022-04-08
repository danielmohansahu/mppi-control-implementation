/* mppi_controller_node.cpp
 *
 * ROS interface for MPPI controller
 */

// STL
#include <mutex>
#include <optional>

// ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <mppi_controller/MPPIOptionsConfig.h>

// custom
#include <mppi_controller/forward_model.h>
#include <mppi_controller/mppi.h>

// convenience typedefs
using geometry_msgs::Twist;
using geometry_msgs::TwistStamped;
using geometry_msgs::PoseStamped;
using nav_msgs::Odometry;
using mppi_controller::MPPIOptionsConfig;

// global options structure
std::shared_ptr<MPPIOptionsConfig> opts2;

// dynamic reconfigure callback
void reconfigure_callback(const MPPIOptionsConfig& opts, uint32_t)
{
  opts2 = std::make_shared<MPPIOptionsConfig>(opts);
  ROS_INFO("Received new dynamic reconfigure request.");
}

// check if we're within our goal radius
inline bool achieved(const PoseStamped& goal, const Odometry& odom, const float radius)
{
  float dist = std::pow(goal.pose.position.x - odom.pose.pose.position.x, 2.0)
               + std::pow(goal.pose.position.y - odom.pose.pose.position.y, 2.0);
  return dist < std::pow(radius, 2.0);
}

// simulate odometry noise
Odometry add_noise(const Odometry& odom)
{
  // @TODO!
  return odom;
}

// simulated controller noise
Twist add_noise(const Twist& cmd)
{
  // @TODO
  return cmd;
}

int main(int argc, char** argv)
{
  // mutex to guard concurrent read / write access
  std::mutex mutex;

  // start ROS node
  ros::init(argc, argv, "mppi_controller_node");
  ros::NodeHandle nh;

  // construct Options struct
  auto options = std::make_shared<mppi::Options>();
  opts2 = std::make_shared<MPPIOptionsConfig>();

  // construct dynamic reconfigure server
  dynamic_reconfigure::Server<MPPIOptionsConfig> server;
  server.setCallback(boost::bind(&reconfigure_callback, _1, _2));

  // construct forward model
  auto forward_model = std::make_shared<mppi::ForwardModel>();
  forward_model->dt = options->dt;
  ROS_INFO_NAMED("mppi_controller_node", "ForwardModel instantiated.");

  // construct base controller
  mppi::MPPI controller(forward_model, nh, options);
  ROS_INFO_NAMED("mppi_controller_node", "MPPI Controller instantiated.");

  // set up twist publisher to send command velocities to robot
  ros::Publisher twist_pub = nh.advertise<Twist>("cmd_vel", 1);
  ros::Publisher twist_debug_pub = nh.advertise<TwistStamped>("cmd_vel_debug", 1);

  // set up subscriber to capture new goals
  std::optional<PoseStamped> current_goal;
  ros::Subscriber goal_sub = nh.subscribe<PoseStamped>(
      "goal",
      1,
      [&current_goal, &controller, &mutex] (const PoseStamped::ConstPtr& msg) -> void
      {
        ROS_DEBUG_NAMED("mppi_controller_node", "Updating goal.");
        std::unique_lock<std::mutex> lock(mutex);
        current_goal = *msg;
        controller.setGoal(*msg);
      });

  // set up subscriber to capture robot odometry
  std::optional<Odometry> current_odometry;
  ros::Subscriber odom_sub = nh.subscribe<Odometry>(
      "odom",
      1,
      [&current_odometry, &mutex] (const Odometry::ConstPtr& msg) -> void
      {
        ROS_DEBUG_NAMED("mppi_controller_node", "Updating odometry.");
        std::unique_lock<std::mutex> lock(mutex);
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
      if (achieved(*current_goal, *current_odometry, options->goal_radius))
      {
        ROS_INFO_NAMED("mppi_controller_node", "Goal achieved!");
        current_goal = std::nullopt;
        controller.clear();

        // send stop command
        twist_pub.publish(Twist());
        continue;
      }

      // if we've made it this far we want to continue planning
      // @TODO figure out how to release lock early
      Twist twist = add_noise(controller.plan(add_noise(*current_odometry)));

      // publish command
      twist_pub.publish(twist);

      // if anyone's listening, publish debug Twist
      if (twist_debug_pub.getNumSubscribers() != 0)
      {
        TwistStamped msg;
        msg.twist = twist;
        msg.header.frame_id = options->frame_id;
        msg.header.stamp = ros::Time::now();
        twist_debug_pub.publish(msg);
      }
    }

    if (ros::Duration delta = (ros::Time::now() - start_time); delta >= ros::Duration(options->dt))
      // we took too long planning... this is very bad
      ROS_WARN_THROTTLE_NAMED(5, "mppi_controller_node", "Planning takes longer than required loop rate!");
    else
      // sleep for remainder of loop
      // @TODO use something better than ros::Time for this?
      (ros::Duration(options->dt) - delta).sleep();
  }

  // shutting down
  return 0;
};
