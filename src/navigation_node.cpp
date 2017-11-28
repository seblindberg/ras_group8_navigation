#include <ros/ros.h>
#include <ras_group8_navigation/Navigation.hpp>

using namespace ras_group8_navigation;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_navigation");
  ros::NodeHandle node_handle("~");
  ros::Rate loop_rate(0.1);
  
  
  /* Create main object */
  Navigation main_object = Navigation::load(node_handle);

  while (node_handle.ok()) {
    ros::spinOnce();
    main_object.update();
    loop_rate.sleep();
  }
  
  main_object.stop();
  
  return 0;
}