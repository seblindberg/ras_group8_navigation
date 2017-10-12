#include <ros/ros.h>
#include <ras_group8_navigation/Navigation.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_navigation");
  ros::NodeHandle nodeHandle("~");

  ras_group8_navigation::Navigation mainObject(nodeHandle);

  ros::spin();
  return 0;
}