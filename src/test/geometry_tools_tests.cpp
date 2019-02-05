#include "ros/ros.h"
#include "costmap_2d/costmap_2d.h"
#include "heatmap/geometry_tools.h"
#include "boost/foreach.hpp"

#include <gtest/gtest.h>

class PointInPolygonTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    //make upright hourglass polygon
    geometry_msgs::Point32 point;
    point.x = -1;
    point.y = 1;
    polygon_.points.push_back(point);
    point.x = 1;
    point.y = 1;
    polygon_.points.push_back(point);
    point.x = -1;
    point.y = -1;
    polygon_.points.push_back(point);
    point.x = 1;
    point.y = -1;
    polygon_.points.push_back(point);
  }
  geometry_msgs::Polygon polygon_;
};

class BottomTopPointPolygonTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    //make upright hourglass polygon
    geometry_msgs::Point32 point;
    point.x = -1;
    point.y = 1;
    polygon_.points.push_back(point);
    point.x = 1;
    point.y = 1;
    polygon_.points.push_back(point);
    point.x = -1;
    point.y = -1;
    polygon_.points.push_back(point);
    point.x = 1;
    point.y = -1;
    polygon_.points.push_back(point);
  }
  geometry_msgs::Polygon polygon_;
};

TEST_F(PointInPolygonTest, outside)
{
  geometry_msgs::Point point;
  point.x = 0.5;
  point.y = 0;
  ASSERT_FALSE(heatmap::pointInPolygon(point, polygon_));
  point.x = -0.5;
  point.y = 0;
  ASSERT_FALSE(heatmap::pointInPolygon(point, polygon_));
  point.x = 0;
  point.y = 1.1;
  ASSERT_FALSE(heatmap::pointInPolygon(point, polygon_));
  point.x = 0;
  point.y = -1.1;
  ASSERT_FALSE(heatmap::pointInPolygon(point, polygon_));
}

TEST_F(PointInPolygonTest, inside)
{
  geometry_msgs::Point point;
  point.x = 0;
  point.y = 0.5;
  ASSERT_TRUE(heatmap::pointInPolygon(point, polygon_));
  point.x = 0;
  point.y = 0.5;
  ASSERT_TRUE(heatmap::pointInPolygon(point, polygon_));
}

TEST_F(BottomTopPointPolygonTest, bottom)
{
  geometry_msgs::Point32 p;
  p.x = -1;
  p.y = -1;

  ASSERT_EQ(heatmap::bottomLeftPointPolygon<geometry_msgs::Point32>(polygon_).x, p.x);
  ASSERT_EQ(heatmap::bottomLeftPointPolygon<geometry_msgs::Point32>(polygon_).y, p.y);
}

TEST_F(BottomTopPointPolygonTest, top)
{
  geometry_msgs::Point32 p;
  p.x = 1;
  p.y = 1;

  ASSERT_EQ(heatmap::topRightPointPolygon<geometry_msgs::Point32>(polygon_).x, p.x);
  ASSERT_EQ(heatmap::topRightPointPolygon<geometry_msgs::Point32>(polygon_).y, p.y);
}

TEST(PointsAdjacentTest, different)
{
  geometry_msgs::Point a, b;
  a.x = 1;
  ASSERT_FALSE(heatmap::pointsNearby(a, b, 0));
  ASSERT_FALSE(heatmap::pointsNearby(a, b, 0.1));
  ASSERT_TRUE(heatmap::pointsNearby(a, b, 1));
}

TEST(PointsAdjacentTest, identical)
{
  geometry_msgs::Point a, b;
  a.x = 1;
  b.x = 1;
  ASSERT_TRUE(heatmap::pointsNearby(a, b, 0));
  ASSERT_TRUE(heatmap::pointsNearby(a, b, 0.1));
  ASSERT_TRUE(heatmap::pointsNearby(a, b, 1));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
