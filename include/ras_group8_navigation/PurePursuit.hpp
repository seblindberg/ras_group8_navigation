#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>

namespace ras_group8_navigation {

typedef struct {
  double x;
  double y;
} Point;

typedef struct {
  Point  A;
  Point  AB;
  double AB_squared;
  double t;
  double d;
} Segment;

class PurePursuit
{
public:
    PurePursuit();
    PurePursuit(const nav_msgs::Path& path_msg, double lookahead);
    
  virtual
    ~PurePursuit();

  geometry_msgs::Pose
    nextPose(const geometry_msgs::Pose& current_pose);
  
private:
  double lookahead_;
  double lookahead_squared_;
  std::vector<Segment> segments_;
  
};

} /* namespace */