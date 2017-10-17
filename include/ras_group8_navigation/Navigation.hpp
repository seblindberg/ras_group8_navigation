#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>

namespace ras_group8_navigation {
  
#define RAS_GROUP8_NAVIGATION_PUBLISH_STATE 1
  
typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> ActionServer;

class Navigation
{
public:
  Navigation(ros::NodeHandle& nodeHandle,
             const std::string& stop_topic,
             const std::string& odom_topic,
             const std::string& cart_topic,
             const std::string& action_topic);
  virtual ~Navigation();

private:
  void stopCallback(const std_msgs::Bool& msg);
  void odomCallback(const nav_msgs::Odometry& msg);
  
  void actionGoalCallback();
  void actionPreemptCallback();

  /* ROS Objects
   */
  ros::NodeHandle& node_handle_;
  ros::Subscriber  stop_subscriber_;
  ros::Subscriber  odom_subscriber_;
  ros::Publisher   cartesian_publisher_;
  
  ActionServer     action_server_;
  
#if RAS_GROUP8_NAVIGATION_PUBLISH_STATE
  ros::Publisher   state_heading_current_publisher_;
  ros::Publisher   state_heading_target_publisher_;
  ros::Publisher   state_distance_publisher_;
#endif
  
  /* Parameters
   */
  //std::string subscriberTopic_;
  
  /* Variables
   */
  geometry_msgs::Pose target_pose_;
};

} /* namespace */