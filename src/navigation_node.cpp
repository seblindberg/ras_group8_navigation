#include <ros/ros.h>
#include <ras_group8_navigation/Navigation.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_navigation");
  ros::NodeHandle node_handle("~");
  ros::Rate loop_rate(0.1);
  
  /* Parameters */
  std::string odom_topic;
  std::string cart_topic;
  
  /* Load parameters */
  if (!node_handle.getParam("odometry_topic", odom_topic))
    return -1;
  if (!node_handle.getParam("cartesian_topic", cart_topic))
    return -1;

  /* Create main object */
  ras_group8_navigation::Navigation main_object(node_handle,
                                                "stop",
                                                odom_topic,
                                                cart_topic,
                                                "navigate");

  while (node_handle.ok()) {
    ros::spinOnce();
    main_object.update();
    loop_rate.sleep();
  }
  return 0;
}