/*
 * Copyright 2015 Fetch Robotics Inc.
 * Author: Michael Ferguson
 */

#include <gtest/gtest.h>
#include <fetch_auto_dock/icp_2d.h>

// TODO: move this to perception.h or something
void getIdealTarget(std::vector<geometry_msgs::Point>& points)
{
  points.clear();

  // Each side is 100mm long, at 45 degree angle
  for (double x = 0.0; x < 0.05 /*0.0707106*/; x += 0.01)
  {
    geometry_msgs::Point p;
    p.x = x;
    p.y = -0.15 - x;
    p.z = 0.0;
    points.push_back(p);
  }
  // Front face is 300mm long
  for (double y = -0.15; y <= 0.15; y += 0.01)
  {
    geometry_msgs::Point p;
    p.x = 0.0; //-0.00487805;
    p.y = y;
    p.z = 0.0;
    points.push_back(p);
  }
  // Each side is 100mm long, at 45 degree angle
  for (double x = 0.0; x < 0.05 /*0.0707106*/; x += 0.001)
  {
    geometry_msgs::Point p;
    p.x = x;
    p.y = 0.15 + x;
    p.z = 0.0;
    points.push_back(p);
  }
}

TEST(TestICP, test_already_aligned)
{
  std::vector<geometry_msgs::Point> source;
  std::vector<geometry_msgs::Point> target;

  getIdealTarget(source);
  getIdealTarget(target);

  geometry_msgs::Transform transform;

  EXPECT_TRUE(icp_2d::alignICP(source, target, transform));
  EXPECT_NEAR(0.0, transform.translation.x, 1e-6);
  EXPECT_NEAR(0.0, transform.translation.y, 1e-6);
  EXPECT_NEAR(0.0, transform.rotation.x, 1e-8);
  EXPECT_NEAR(0.0, transform.rotation.y, 1e-8);
  EXPECT_NEAR(0.0, transform.rotation.z, 1e-6);
  EXPECT_NEAR(1.0, transform.rotation.w, 1e-6);
}

TEST(TestICP, test_shifted)
{
  std::vector<geometry_msgs::Point> source;
  std::vector<geometry_msgs::Point> target;

  getIdealTarget(source);
  getIdealTarget(target);
  source = icp_2d::transform(source, 0.1, 0.1, 0);

  geometry_msgs::Transform transform;

  EXPECT_TRUE(icp_2d::alignICP(source, target, transform));
  EXPECT_NEAR(-0.1, transform.translation.x, 0.002);
  EXPECT_NEAR(-0.1, transform.translation.y, 0.002);
  EXPECT_NEAR(0.0, transform.rotation.x, 1e-8);
  EXPECT_NEAR(0.0, transform.rotation.y, 1e-8);
  EXPECT_NEAR(0.0, transform.rotation.z, 1e-8);
  EXPECT_NEAR(1.0, transform.rotation.w, 1e-8);
}

TEST(TestICP, test_rotated)
{
  std::vector<geometry_msgs::Point> source;
  std::vector<geometry_msgs::Point> target;

  getIdealTarget(source);
  getIdealTarget(target);
  source = icp_2d::transform(source, 0, 0, 0.2);

  geometry_msgs::Transform transform;

  EXPECT_TRUE(icp_2d::alignICP(source, target, transform));
  EXPECT_NEAR(0.0, transform.translation.x, 0.002);
  EXPECT_NEAR(0.0, transform.translation.y, 0.002);
  EXPECT_NEAR(0.0, transform.rotation.x, 1e-8);
  EXPECT_NEAR(0.0, transform.rotation.y, 1e-8);
  double theta = icp_2d::thetaFromQuaternion(transform.rotation);
  EXPECT_NEAR(-0.2, theta, 1e-2);
}

TEST(TestICP, test_shifted_rotated)
{
  std::vector<geometry_msgs::Point> source, source_t;
  std::vector<geometry_msgs::Point> target;

  getIdealTarget(source);
  getIdealTarget(target);
  source_t = icp_2d::transform(source, 0.1, 0.2, 0.3);

  geometry_msgs::Transform transform;
  EXPECT_TRUE(icp_2d::alignICP(source_t, target, transform));
  EXPECT_NEAR(0.0, transform.rotation.x, 1e-8);
  EXPECT_NEAR(0.0, transform.rotation.y, 1e-8);
  double theta = icp_2d::thetaFromQuaternion(transform.rotation);
  EXPECT_NEAR(-0.3, theta, 1e-3);

  source_t = icp_2d::transform(source_t,
                               transform.translation.x,
                               transform.translation.y,
                               icp_2d::thetaFromQuaternion(transform.rotation));

  for (size_t i = 0; i < source.size(); i++)
  {
    EXPECT_NEAR(source[i].x, source_t[i].x, 0.002);
    EXPECT_NEAR(source[i].y, source_t[i].y, 0.002);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
