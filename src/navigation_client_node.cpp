#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <signal.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ActionClient;

static ActionClient* action_client;

void quit(int sig)
{
  if (action_client != NULL && action_client->isServerConnected()) {
    action_client->cancelGoal();
    ROS_INFO("Canceling goal");
  }
  
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_navigation_client");
  ros::NodeHandle node_handle("~");
  
  double x;
  double y;
  std::string server;
  
  if (!node_handle.getParam("x", x)) {
    ROS_ERROR("Failed to read x parameter");
    exit(-1);
  }
  
  if (!node_handle.getParam("y", y)) {
    ROS_ERROR("Failed to read y parameter");
    exit(-1);
  }
  
  if (!node_handle.getParam("server", server)) {
    ROS_ERROR("Failed to read server parameter");
    exit(-1);
  }
  
  /* Tell the action client that we want to spin a thread by default */
  ActionClient ac(server, true);
  
  action_client = &ac;
  /* Trap sigint */
  signal(SIGINT, quit);
  
  /* Wait for the action server to come up */
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the action server to come up");
  }
  
  /* Setup the goal pose */
  move_base_msgs::MoveBaseGoal goal;
  
  /* Send the target pose in the odom frame */
  goal.target_pose.header.frame_id = "odom";
  goal.target_pose.header.stamp = ros::Time::now();
  
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  
  /* Let the other thread run */
  ac.waitForResult();
  
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Success");
  else
    ROS_INFO("Failure");
  
  return 0;
}