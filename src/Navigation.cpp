#include <ras_group8_navigation/Navigation.hpp>

// STD
#include <string>

namespace ras_group8_navigation {

Navigation::Navigation(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,
                                      &Navigation::topicCallback, this);

  ROS_INFO("Successfully launched node.");
}

Navigation::~Navigation()
{
}

bool Navigation::readParameters()
{
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
  return true;
}

void Navigation::topicCallback(const phidgets::motor_encoder& msg)
{
}


} /* namespace */