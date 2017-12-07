#include <ros/ros.h>
#include <ras_group8_navigation/Navigation.hpp>

using namespace ras_group8_navigation;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_navigation");
  ros::NodeHandle node_handle("~");
  const double update_rate = 1;

  /* Create main object */
  Navigation main_object = Navigation::load(node_handle);

  main_object.run(update_rate);
    
  ros::spin();
  
  ROS_INFO("Stoping");
  main_object.stop();
  
  return 0;
}