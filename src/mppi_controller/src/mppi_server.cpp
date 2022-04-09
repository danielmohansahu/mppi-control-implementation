/* mppi_server.cpp
 *
 * Implementation of ROS interface for MPPI.
 */

#include <mppi_controller/mppi_server.h>

namespace mppi
{

MPPIServer::MPPIServer(ros::NodeHandle nh)
  : opts_(std::make_shared<MPPIOptionsConfig>()),
    waypoint_server_(nh, "waypoint", boost::bind(&MPPIServer::executeWP, this, _1), false),
    follow_course_server_(nh, "follow_course", boost::bind(&MPPIServer::executeFC, this, _1), false)
{
  ROS_INFO_NAMED("MPPIServer", "Controller instantiated, bringing up server...");

  // instantiate dynamic reconfigure server
  //  this needs to be done first to ensure all options are set to their correct values
  //  e.g. not everything is dynamic!
  reconfigure_server_.setCallback(
    boost::bind(&MPPIServer::reconfigure_callback, this, _1, _2));

  // initialize forward model
  model_ = std::make_shared<ForwardModel>(opts_);

  // initialize controller itself
  controller_ = std::make_unique<MPPI>(model_, nh, opts_);

  // instantiate ROS publishers
  twist_pub_ = nh.advertise<Twist>("cmd_vel", 1);
  twist_debug_pub_ = nh.advertise<TwistStamped>("cmd_vel_debug", 1);

  // set up subscriber to capture robot odometry
  odom_sub_ = nh.subscribe<Odometry>("odom", 1,
    [this] (const Odometry::ConstPtr& msg) -> void
    {
      ROS_DEBUG_NAMED("MPPIServer", "Updating odometry.");
      std::unique_lock<std::mutex> lock(mutex_);
      current_odometry_ = *msg;
    });
  
  // refuse to start until we've got a good odometry estimate
  while (ros::ok())
    if (std::unique_lock<std::mutex> lock(mutex_); !current_odometry_)
      ROS_WARN_THROTTLE_NAMED(5, "mppi_controller_node", "Waiting for initial odometry.");
    else
      break;

  // initialize and start action server
  ROS_INFO_NAMED("MPPIServer", "Starting action servers.");
  waypoint_server_.start();
  follow_course_server_.start();

  ROS_INFO_NAMED("MPPIServer", "MPPI server instantiated. Ready to execute goals.");
}

void MPPIServer::reconfigure_callback(MPPIOptionsConfig opts, uint32_t)
{
  *opts_ = std::move(opts);
  ROS_INFO("Received new dynamic reconfigure request.");
  
  // sanity check via spot-checks
  assert(opts_->wheel_radius != 0 && "Something went wrong with dynamic reconfigure.");
  assert(opts_->rollouts != 0 && "Something went wrong with dynamic reconfigure.");
}

nav_msgs::Odometry MPPIServer::add_noise(const Odometry& odom)
{
  // @TODO!
  return odom;
}

geometry_msgs::Twist MPPIServer::add_noise(const Twist& cmd)
{
  // @TODO
  return cmd;
}

void MPPIServer::publish(const Twist& twist)
{
  // publish command and, optionally, debugging information
  twist_pub_.publish(twist);

  // if anyone's listening, publish debug Twist
  if (twist_debug_pub_.getNumSubscribers() != 0)
  {
    TwistStamped msg;
    msg.twist = twist;
    msg.header.frame_id = opts_->frame_id;
    msg.header.stamp = ros::Time::now();
    twist_debug_pub_.publish(msg);
  }
}

void MPPIServer::executeWP(const WaypointGoal::ConstPtr& goal)
{
  ROS_INFO_NAMED("MPPIServer", "Received Waypoint goal!");

  // perform sanity checks
  assert(current_odometry_ && "No odometry information found. Can't execute goal.");

  // set new goal for controller
  controller_->setGoal(*goal);

  // initialize result
  WaypointResult result;
  result.success = true;

  // plan until goal succeeds
  ros::Time start_time = ros::Time::now();
  ROS_INFO_NAMED("MPPIServer", "Beginning planning loop...");
  while (ros::ok())
  {
    // begin loop timing
    ros::Time loop_time = ros::Time::now();

    // make a copy of current odometry
    Odometry odom;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      odom = *current_odometry_;
    }

    // check if we've achieved our goal
    if (waypoint_achieved(goal->pose, odom, goal->radius))
    {
      ROS_INFO_NAMED("MPPIServer", "Goal achieved!");
      result.success = true;
      break;
    }

    // check if we've timed out
    if (goal->timeout > 0 && (ros::Time::now() - start_time).toSec() > goal->timeout)
    {
      ROS_INFO_STREAM_NAMED("MPPIServer", "Timed out (" << goal->timeout << " s limit).");
      result.success = false;
      break;
    }

    // check if we've been preempted
    if (waypoint_server_.isPreemptRequested())
    {
      ROS_INFO_NAMED("MPPIServer", "Preempted!");
      result.success = false;
      break;
    }

    // if we've made it this far we want to continue planning
    publish(add_noise(controller_->plan(add_noise(odom))));

    // sleep for the appropriate duration
    if (ros::Duration delta = (ros::Time::now() - loop_time); delta >= ros::Duration(opts_->dt))
      // we took too long planning... this is very bad
      ROS_WARN_THROTTLE_NAMED(5, "MPPIServer", "Planning takes longer than required loop rate!");
    else
      // sleep for remainder of loop
      // @TODO use something better than ros::Time for this?
      (ros::Duration(opts_->dt) - delta).sleep();
  }

  // always send a stop command and clear the controller before exiting
  controller_->clear();
  publish(Twist());

  // if successful, let everyone know
  if (result.success)
    waypoint_server_.setSucceeded(result);
}

void MPPIServer::executeFC(const FollowCourseGoal::ConstPtr& goal)
{
  ROS_INFO_NAMED("MPPIServer", "Received FollowCourse goal!");

  // perform sanity checks
  assert(current_odometry_ && "No odometry information found. Can't execute goal.");
  if (goal->duration <= 0)
  {
    ROS_ERROR_NAMED("MPPIServer", "Received invalid goal; duration must be greater than zero.");
    follow_course_server_.setAborted();
    return;
  }

  // set new goal for controller
  controller_->setGoal(*goal);

  // plan until time's up
  bool success = true;
  ros::Time start_time = ros::Time::now();
  ROS_INFO_NAMED("MPPIServer", "Beginning planning loop...");
  while (ros::ok() && (ros::Time::now() - start_time).toSec() < goal->duration)
  {
    // begin loop timing
    ros::Time loop_time = ros::Time::now();

    // make a copy of current odometry
    Odometry odom;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      odom = *current_odometry_;
    }

    // check if we've been preempted
    if (follow_course_server_.isPreemptRequested())
    {
      ROS_INFO_NAMED("MPPIServer", "Preempted!");
      success = false;
      break;
    }

    // if we've made it this far we want to continue planning
    publish(add_noise(controller_->plan(add_noise(odom))));

    // sleep for the appropriate duration
    if (ros::Duration delta = (ros::Time::now() - loop_time); delta >= ros::Duration(opts_->dt))
      // we took too long planning... this is very bad
      ROS_WARN_THROTTLE_NAMED(5, "MPPIServer", "Planning takes longer than required loop rate!");
    else
      // sleep for remainder of loop
      // @TODO use something better than ros::Time for this?
      (ros::Duration(opts_->dt) - delta).sleep();
  }

  // always send a stop command and clear the controller before exiting
  controller_->clear();
  publish(Twist());

  // we are pretty much always successful, unless preempted
  if (success)
    follow_course_server_.setSucceeded();
}

} // namespace mppi
