#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <ras_group8_navigation/PurePursuit.hpp>

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

  void
    update();

private:
  bool
    requestPath(const geometry_msgs::PoseStamped& current_pose,
                const geometry_msgs::PoseStamped& target_pose);
  
  void
    publishTwist(double x, double w);
    
  double
    poseToHeading(const geometry_msgs::Pose& pose);
    
  void
    goalCallback(const geometry_msgs::PoseStamped& msg);
    
  void
    stopCallback(const std_msgs::Bool& msg);
    
  void
    odomCallback(const nav_msgs::Odometry& msg);
  
  void
    actionGoalCallback();
    
  void
    actionPreemptCallback();

  /* ROS Objects
   */
  ros::NodeHandle& node_handle_;
  ros::Subscriber  goal_subscriber_;
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
  
  /* Service Clients
   */
  ros::ServiceClient path_service_;
  
  /* Variables
   */
  nav_msgs::Path path_; /* TODO: we don't really need to store the path */
  geometry_msgs::Pose target_pose_;
  geometry_msgs::Pose current_pose_;
  
  PurePursuit pure_pursuit_;
};

} /* namespace */