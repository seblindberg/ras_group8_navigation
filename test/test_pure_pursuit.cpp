#include <gtest/gtest.h>
#include <ras_group8_navigation/PurePursuit.hpp>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>

using namespace ras_group8_navigation;

TEST(PurePursuit, simple_path)
{
  nav_msgs::Path      path;
  geometry_msgs::Pose pose;
  
  path.poses.resize(2);
  path.poses[0].pose.position.x = 0.0;
  path.poses[0].pose.position.y = 0.0;
  path.poses[1].pose.position.x = 1.0;
  path.poses[1].pose.position.y = 0.75;
  
  pose.position.x = 0.1;
  pose.position.y = 0.2;
  
  PurePursuit pp(path, 0.3);
  
  geometry_msgs::Pose next_pose = pp.nextPose(pose);
  
  EXPECT_FLOAT_EQ(0.38627417, next_pose.position.x);
  EXPECT_FLOAT_EQ(0.28970563, next_pose.position.y);
}

TEST(PurePursuit, longer_path)
{
  nav_msgs::Path      path;
  geometry_msgs::Pose pose;

  path.poses.resize(6);
  path.poses[0].pose.position.x =  2.00;
  path.poses[0].pose.position.y =  1.00;
  path.poses[1].pose.position.x =  1.25;
  path.poses[1].pose.position.y =  1.75;
  path.poses[2].pose.position.x =  5.25;
  path.poses[2].pose.position.y =  8.25;
  path.poses[3].pose.position.x =  7.25;
  path.poses[3].pose.position.y =  8.75;
  path.poses[4].pose.position.x = 11.75;
  path.poses[4].pose.position.y = 10.75;
  path.poses[5].pose.position.x = 12.00;
  path.poses[5].pose.position.y = 10.00;

  pose.position.x = 3.0;
  pose.position.y = 2.0;

  PurePursuit pp(path, 2.0);

  geometry_msgs::Pose next_pose = pp.nextPose(pose);

  EXPECT_FLOAT_EQ(2.61112619, next_pose.position.x);
  EXPECT_FLOAT_EQ(3.96183005, next_pose.position.y);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}