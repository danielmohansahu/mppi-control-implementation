/* mppi_server.h
 *
 * Core execution server for MPPI.
 *
 * This implements action servers around MPPI
 * to support various goal execution procedures.
 */

// STL
#include <mutex>
#include <optional>

// ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <mppi_controller/MPPIOptionsConfig.h>
#include <mppi_controller/WaypointAction.h>

// custom
#include <mppi_controller/forward_model.h>
#include <mppi_controller/mppi.h>

namespace mppi
{

// check if we're within our goal radius
inline bool waypoint_achieved(const geometry_msgs::PoseStamped& goal,
                              const nav_msgs::Odometry& odom,
                              const float radius)
{
  float dist = std::pow(goal.pose.position.x - odom.pose.pose.position.x, 2.0)
               + std::pow(goal.pose.position.y - odom.pose.pose.position.y, 2.0);
  return dist < std::pow(radius, 2.0);
}

class MPPIServer
{
 public:
  // convenience typedefs
  using Twist             = geometry_msgs::Twist;
  using TwistStamped      = geometry_msgs::TwistStamped;
  using PoseStamped       = geometry_msgs::PoseStamped;
  using Odometry          = nav_msgs::Odometry;
  using MPPIOptionsConfig = mppi_controller::MPPIOptionsConfig;
  using WaypointAction    = mppi_controller::WaypointAction;
  using WaypointGoal      = mppi_controller::WaypointGoal;
  using WaypointResult    = mppi_controller::WaypointResult;
  using WaypointFeedback  = mppi_controller::WaypointFeedback;

  // expected constructor
  MPPIServer(ros::NodeHandle nh);

 private:
  
  // execution callback for Waypoint based goals
  void execute(const WaypointGoal::ConstPtr& goal);

  // dynamic reconfigure callback
  void reconfigure_callback(const MPPIOptionsConfig& opts, uint32_t);

  // simulate odometry noise
  Odometry add_noise(const Odometry& odom);

  // simulated controller noise
  Twist add_noise(const Twist& cmd);

  // publish command velocity
  void publish(const Twist& twist);

  // shared options structure
  std::shared_ptr<MPPIOptionsConfig> opts_;

  // core MPPI forward model
  std::shared_ptr<ForwardModel> model_;
  
  // core MPPI controller
  std::unique_ptr<MPPI> controller_;

  // locking protection
  std::mutex mutex_;

  // waypoint action server
  actionlib::SimpleActionServer<WaypointAction> waypoint_server_;

  // dynamic reconfigure server
  dynamic_reconfigure::Server<MPPIOptionsConfig> reconfigure_server_;

  // ROS interface objects
  ros::Publisher twist_pub_, twist_debug_pub_;
  ros::Subscriber odom_sub_;

  // latest odometry estimate
  std::optional<Odometry> current_odometry_;

};

} // namespace mppi
