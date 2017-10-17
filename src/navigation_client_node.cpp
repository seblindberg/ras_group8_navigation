#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
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

void goalCallback(const geometry_msgs::PoseStamped& msg)
{
  double x;
  double y;
  
  if (action_client == NULL) {
    ROS_ERROR("No Action Client");
    exit(-1);
  }
  
  /* Setup the goal pose */
  move_base_msgs::MoveBaseGoal goal;
  
  /* Send the target pose in the odom frame */
  goal.target_pose = msg;
  
  ROS_INFO("Sending goal");
  action_client->sendGoal(goal);
  
  /* Let the other thread run */
  action_client->waitForResult();
  
  if(action_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Success");
  } else {
    ROS_INFO("Failure");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_navigation_client");
  ros::NodeHandle node_handle("~");
  
  ros::Subscriber goal_subscriber =
    node_handle.subscribe("/move_base_simple/goal", 1,
                          &goalCallback);
  
  std::string server;
    
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
  
  ROS_INFO("Connected");
  
  ros::spin();
  
  return 0;
}