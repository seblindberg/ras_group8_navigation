#include <ras_group8_navigation/Navigation.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
//#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

// STD
#include <string.h>
#include <math.h>

namespace ras_group8_navigation {

Navigation::Navigation(ros::NodeHandle& node_handle,
                       const std::string& stop_topic,
                       const std::string& odom_topic,
                       const std::string& cart_topic)
    : node_handle_(node_handle),
      action_server_(node_handle, "navigate", false)
{
  stop_subscriber_ =
    node_handle_.subscribe(stop_topic, 1,
                           &Navigation::stopCallback, this);
  odom_subscriber_ =
    node_handle_.subscribe(odom_topic, 1,
                           &Navigation::odomCallback, this);
                          
  cartesian_publisher_ =
    node_handle_.advertise<geometry_msgs::Twist>(cart_topic, 1);

  /* Register the action callbacks */
  action_server_.registerGoalCallback(
    boost::bind(&Navigation::actionGoalCallback, this));
    
  action_server_.registerPreemptCallback(
    boost::bind(&Navigation::actionPreemptCallback, this));

  action_server_.start();

  ROS_INFO("Successfully launched node.");
}

Navigation::~Navigation()
{
  odom_subscriber_.shutdown();
  cartesian_publisher_.shutdown();
}

void Navigation::stopCallback(const std_msgs::Bool& msg)
{
  actionPreemptCallback();
}

void Navigation::odomCallback(const nav_msgs::Odometry& msg)
{
  /* Make sure we are running */
  if (!action_server_.isActive()) {
    ROS_INFO("No target");
    return;
  }
    
  /* Extract x, y and yaw from the msg */
  double x = msg.pose.pose.position.x;
  double y = msg.pose.pose.position.y;
  
  double dx = target_pose_.position.x - x;
  double dy = target_pose_.position.y - y;
  
  /* Check distance to target */
  double d = sqrt(dx*dx + dy*dy);
  
  ROS_INFO("Distance to target: %f", d);
  
  if (d < 0.1) {
    /* Indicate that we are done */
    action_server_.setSucceeded();
    return;
  }
  
  /* Convert the orientation to a tf Quaternion */
  tf::Quaternion q(msg.pose.pose.orientation.x,
                   msg.pose.pose.orientation.y,
                   msg.pose.pose.orientation.z,
                   msg.pose.pose.orientation.w);
  
  double yaw;
  double pitch;
  double roll;
  tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
    
  /* Calculate delta angle to the target pose */
  double dtheta = atan2(dy, dx);
  
  /* Steer towords the target pose */
  geometry_msgs::Twist twist_msg;
  
  twist_msg.linear.x  = 0.25; /* TODO: set as parameter */
  twist_msg.angular.z = (dtheta - yaw) * 10; /* 10 hz */
  
  if (twist_msg.angular.z < -0.1) {
    twist_msg.angular.z = -0.1; /* Min angular velocity */
  } else if (twist_msg.angular.z > 0.1) {
    twist_msg.angular.z =  10.1; /* Max angular velocity */
  }
  
  cartesian_publisher_.publish(twist_msg);
}

void Navigation::actionGoalCallback()
{
  ROS_INFO("Received target pose");
  target_pose_ = action_server_.acceptNewGoal()->target_pose.pose;
  
  geometry_msgs::Twist twist_msg;
  
  twist_msg.linear.x = 0.2;
  twist_msg.angular.z = 0;
  
  cartesian_publisher_.publish(twist_msg);
}

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