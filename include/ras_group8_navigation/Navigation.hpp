#pragma once

#include <ros/ros.h>
#include <phidgets/motor_encoder.h>

namespace ras_group8_navigation {

class Navigation
{
public:
  Navigation(ros::NodeHandle& nodeHandle);
  virtual ~Navigation();

private:
  bool readParameters();
  void topicCallback(const phidgets::motor_encoder& msg);

  /* ROS Objects
   */
  ros::NodeHandle& nodeHandle_;
  ros::Subscriber subscriber_;
  
  /* Parameters
   */
  std::string subscriberTopic_;
};

} /* namespace */