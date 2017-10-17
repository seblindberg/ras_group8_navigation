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
                       const std::string& stop_topic,
                       const std::string& odom_topic,
                       const std::string& cart_topic,
                       const std::string& action_topic)
    : node_handle_(node_handle),
      action_server_(node_handle, action_topic, false)
{
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

  /* Register the action callbacks */
  action_server_.registerGoalCallback(
    boost::bind(&Navigation::actionGoalCallback, this));
    
  action_server_.registerPreemptCallback(
    boost::bind(&Navigation::actionPreemptCallback, this));
    
#if RAS_GROUP8_NAVIGATION_PUBLISH_STATE
  state_heading_current_publisher_ =
    node_handle_.advertise<std_msgs::Float32>("heading_current", 1);
    
  state_heading_target_publisher_ =
    node_handle_.advertise<std_msgs::Float32>("heading_target", 1);
    
  state_distance_publisher_ =
    node_handle_.advertise<std_msgs::Float32>("distance", 1);
    
  ROS_INFO("Compiled with state output");
#endif

  action_server_.start();

  ROS_INFO("Successfully launched node.");
}

Navigation::~Navigation()
{
}

/* Publish Twist
 * Private method for publishing linear and angular velocity to the cartesian
 * controller.
 */
void Navigation::publishTwist(double x, double w)
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
double Navigation::poseToHeading(const geometry_msgs::Pose& pose)
{
  double yaw;
  double pitch;
  double roll;
  
  /* Convert the orientation to a tf Quaternion */
  tf::Quaternion q(pose.orientation.x,
                   pose.orientation.y,
                   pose.orientation.z,
                   pose.orientation.w);
  
  tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
  
  return yaw;
}

/* Stop Callback
 * Called when a boolean message is published to the stop topic.
 *
 * Stops the current navigation task.
 */
void Navigation::stopCallback(const std_msgs::Bool& msg)
{
  actionPreemptCallback();
}

/* Odom Callback
 * Called when a new pose is published from the odometry.
 *
 * Calculates the difference between the current pose and the target pose and
 * steers the robot to it using messages to the cartesian controller.
 */
void Navigation::odomCallback(const nav_msgs::Odometry& msg)
{
  /* Make sure we are running */
  if (!action_server_.isActive()) {
    ROS_INFO("No target");
    return;
  }
  
  double dx = target_pose_.position.x - msg.pose.pose.position.x;
  double dy = target_pose_.position.y - msg.pose.pose.position.y;
  
  /* Check distance to target */
  double d = sqrt(dx*dx + dy*dy);
  
  ROS_INFO("Distance to target: %f", d);
  
  if (d < 0.05) { /* TODO: Extract as parameter */
    /* Indicate that we are done */
    action_server_.setSucceeded();
    publishTwist(0, 0);
    return;
  }
  
  double yaw = poseToHeading(msg.pose.pose);
  
  /* Calculate delta angle to the target pose */
  double dtheta = atan2(dy, dx);
    
  double w;
  
  w = (yaw - dtheta) * 10; /* 10 hz */
  
  /* TODO: Extract as parameters */
  if (w < -0.5) {
    w = -0.5; /* Min angular velocity */
  } else if (w > 0.5) {
    w =  0.5; /* Max angular velocity */
  }
  
  publishTwist(0.2, w); /* TODO: set as parameter */
  
#if RAS_GROUP8_NAVIGATION_PUBLISH_STATE
  std_msgs::Float32 state_msg;

  state_msg.data = yaw;
  state_heading_current_publisher_.publish(state_msg);

  state_msg.data = dtheta;
  state_heading_target_publisher_.publish(state_msg);

  state_msg.data = d;
  state_distance_publisher_.publish(state_msg);
#endif
}

/* Goal Callback
 * Called by actionlib when a new target pose is received.
 *
 * When a new target pose is received the old pose is replaced. This behaviour
 * may change in future versions.
 */
void Navigation::actionGoalCallback()
{
  ROS_INFO("Received target pose");
  target_pose_ = action_server_.acceptNewGoal()->target_pose.pose;
  
  geometry_msgs::Twist twist_msg;
  
  twist_msg.linear.x = 0.1;
  twist_msg.angular.z = 0;
  
  cartesian_publisher_.publish(twist_msg);
}

/* Preempt Callback
 * Called by actionlib when the current goal is cancelled.
 *
 * The cartesian controller is updated with zero linear and angular velocity.
 */
void Navigation::actionPreemptCallback()
{
  ROS_INFO("Canceling goal");
  action_server_.setPreempted();
  
  /* Tell the cartesian controller to stop */
  geometry_msgs::Twist twist_msg;
  
  twist_msg.linear.x = 0;
  twist_msg.angular.z = 0;
  
  cartesian_publisher_.publish(twist_msg);
}


} /* namespace */