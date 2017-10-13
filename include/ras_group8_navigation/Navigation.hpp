#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>

namespace ras_group8_navigation {
  
typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> ActionServer;

class Navigation
{
public:
  Navigation(ros::NodeHandle& nodeHandle,
             const std::string& odom_topic,
             const std::string& cart_topic);
  virtual ~Navigation();

private:
  void odomCallback(const nav_msgs::Odometry& msg);
  
  void actionGoalCallback();
  void actionPreemptCallback();

  /* ROS Objects
   */
  ros::NodeHandle& node_handle_;
  ros::Subscriber  odom_subscriber_;
  ros::Publisher   cartesian_publisher_;
  
  ActionServer     action_server_;
  
  /* Parameters
   */
  //std::string subscriberTopic_;
  
  /* Variables
   */
  geometry_msgs::Pose target_pose_;
};

} /* namespace */