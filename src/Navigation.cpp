#include <ras_group8_navigation/Navigation.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/LinearMath/Matrix3x3.h>

#if RAS_GROUP8_NAVIGATION_PUBLISH_STATE
#include <std_msgs/Float32.h>
#endif

// STD
#include <string.h>
#include <math.h>

namespace ras_group8_navigation {

Navigation::Navigation(ros::NodeHandle& node_handle,
                       const std::string& goal_topic,
                       const std::string& stop_topic,
                       const std::string& odom_topic,
                       const std::string& cart_topic,
                       double pursuit_radius)
    : node_handle_(node_handle),
      is_active_(false),
      pure_pursuit_radius_(pursuit_radius),
      update_rate_(10)
{
  goal_subscriber_ =
    node_handle_.subscribe(goal_topic, 1,
                           &Navigation::goalCallback, this);
  
  /* Listen for a boolean message to stop the current trajectory. */
  stop_subscriber_ =
    node_handle_.subscribe(stop_topic, 1,
                           &Navigation::stopCallback, this);

  /* Listen to updates from the odometry. */
  odom_subscriber_ =
    node_handle_.subscribe(odom_topic, 1,
                           &Navigation::odomCallback, this);

  /* Publish cartesian twist messages to drive the vehicle */
  cartesian_publisher_ =
    node_handle_.advertise<geometry_msgs::Twist>(cart_topic, 1);

    
#if RAS_GROUP8_NAVIGATION_PUBLISH_STATE
  state_heading_current_publisher_ =
    node_handle_.advertise<std_msgs::Float32>("heading_current", 1);
    
  state_heading_target_publisher_ =
    node_handle_.advertise<std_msgs::Float32>("heading_target", 1);
    
  state_distance_publisher_ =
    node_handle_.advertise<std_msgs::Float32>("distance", 1);
    
  ROS_INFO("Compiled with state output");
#endif

  ROS_INFO("Successfully launched node.");
}

Navigation::~Navigation()
{
}

void
Navigation::update()
{
  if (!is_active_) {
    return;
  }
  
  geometry_msgs::Pose target_pose = pure_pursuit_.nextPose(current_pose_);
  
  const double dx = target_pose.position.x - current_pose_.position.x;
  const double dy = target_pose.position.y - current_pose_.position.y;

  /* Check distance to target */
  const double d = sqrtf(dx*dx + dy*dy);
  ROS_INFO("Distance to target: %f", d);
  
  const double yaw = poseToHeading(current_pose_);
  double target_yaw;
  
  double v;
  double w;
  
  /* If we are close enough we want to assume the correct
     heading of the target */
  if (d < 0.05) { /* TODO: Extract as parameter */
    target_yaw = poseToHeading(target_pose);
    
    v = 0;
    w = (yaw - target_yaw) * update_rate_;
    
    if (fabs(w) < 0.05) {
      stop();
      return;
    }
  } else {
    /* Calculate the yaw angle to the target pose */
    target_yaw = atan2(dy, dx);
    
    
    v = 0.2; /* Use constant velocity */
    w = (yaw - target_yaw) * update_rate_; /* 10 hz */
  }

  /* TODO: Extract as parameters */
  if (w < -0.5) {
    w = -0.5; /* Min angular velocity */
  } else if (w > 0.5) {
    w =  0.5; /* Max angular velocity */
  }

  publishTwist(v, w); /* TODO: set as parameter */

#if RAS_GROUP8_NAVIGATION_PUBLISH_STATE
  std_msgs::Float32 state_msg;

  state_msg.data = yaw;
  state_heading_current_publisher_.publish(state_msg);

  state_msg.data = target_yaw;
  state_heading_target_publisher_.publish(state_msg);

  state_msg.data = d;
  state_distance_publisher_.publish(state_msg);
#endif
}

void
Navigation::stop()
{
  is_active_ = false;
  publishTwist(0, 0);
}


/* Publish Twist
 * Private method for publishing linear and angular velocity to the cartesian
 * controller.
 */
void
Navigation::publishTwist(double x, double w)
{
  geometry_msgs::Twist twist_msg;
  
  twist_msg.linear.x  = x;
  twist_msg.angular.z = w;
  
  cartesian_publisher_.publish(twist_msg);
}

/* Pose To Heading
 * Private method for extracting the heading from a PoseStamped message.
 *
 * Converts from quaternion quardinates to yaw, expressed in radians.
 */
double
Navigation::poseToHeading(const geometry_msgs::Pose& pose)
{
  double yaw;
  double tmp;
  
  /* Convert the orientation to a tf Quaternion */
  tf::Quaternion q(pose.orientation.x,
                   pose.orientation.y,
                   pose.orientation.z,
                   pose.orientation.w);
  
  tf::Matrix3x3(q).getEulerYPR(yaw, tmp, tmp);
  
  return yaw;
}

/* Stop Callback
 * Called when a boolean message is published to the stop topic.
 *
 * Stops the current navigation task.
 */
void
Navigation::stopCallback(const std_msgs::Bool& msg)
{
  ROS_INFO("Stoping");
  
  stop();
}

/* Odom Callback
 *
 * Called when a new pose is published from the odometry.
 *
 * Calculates the difference between the current pose and the target pose and
 * steers the robot to it using messages to the cartesian controller.
 */
void
Navigation::odomCallback(const nav_msgs::Odometry& msg)
{
  /* TODO: Make sure the coordinates are transformed to the correct frame */
  current_pose_ = msg.pose.pose;
}

/* Goal Callback
 *
 */
void
Navigation::goalCallback(const nav_msgs::Path& msg)
{
  ROS_INFO("Received path");
  
  pure_pursuit_ = PurePursuit(msg, pure_pursuit_radius_);
  is_active_    = true;
}

Navigation
Navigation::load(ros::NodeHandle& n)
{
  /* Load optional parameters */
  const std::string goal_topic =
    n.param("goal_topic", std::string("path"));
    
  const std::string stop_topic =
    n.param("stop_topic", std::string("stop"));
    
  const std::string odom_topic =
    n.param("odom_topic", std::string("odom"));
    
  const std::string cart_topic =
    n.param("cart_topic", std::string("cart"));
    
  const double pursuit_radius =
    n.param("pursuit_radius", 0.1);
  
  Navigation object(n, goal_topic,
                       stop_topic,
                       odom_topic,
                       cart_topic,
                       pursuit_radius);
  
  return object;
}

} /* namespace */