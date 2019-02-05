/*
 * points_to_polygon.cpp
 *
 *  Created on: 13 May 2015
 *      Author: baueraff
 */
#include "ros/ros.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <heatmap/geometry_tools.h>

#include <visualization_msgs/Marker.h>

#include <boost/foreach.hpp>
#include <ros/wall_timer.h>

geometry_msgs::PolygonStamped input_;
ros::Publisher point_viz_pub_;
ros::Publisher polygon_planner_pub;
ros::WallTimer point_viz_timer_;

void vizPubCb()
{
  visualization_msgs::Marker points, line_strip;
  points.header = line_strip.header = input_.header;
  points.ns = line_strip.ns = "explore_points";
  points.id = 0;
  line_strip.id = 1;
  points.type = visualization_msgs::Marker::SPHERE_LIST;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  if (!input_.polygon.points.empty())
  {
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    points.scale.x = points.scale.y = 0.1;
    line_strip.scale.x = 0.05;
    BOOST_FOREACH(geometry_msgs::Point32 point, input_.polygon.points){
    line_strip.points.push_back(costmap_2d::toPoint(point));
    points.points.push_back(costmap_2d::toPoint(point));
  }
    if (false)
    {
      line_strip.points.push_back(costmap_2d::toPoint(input_.polygon.points.front()));
      points.color.a = points.color.r = line_strip.color.r = line_strip.color.a = 1.0;
    }
    else
    {
      points.color.a = points.color.b = line_strip.color.b = line_strip.color.a = 1.0;
    }
  }
  else
  {
    points.action = line_strip.action = visualization_msgs::Marker::DELETE;
  }
  point_viz_pub_.publish(points);
  point_viz_pub_.publish(line_strip);
}

void pointCb(const geometry_msgs::PointStampedConstPtr& point)
{
  double average_distance = heatmap::polygonPerimeter(input_.polygon) / input_.polygon.points.size();

  if (input_.polygon.points.empty())
  {
    ROS_INFO("First point!");
    input_.header = point->header;
    input_.polygon.points.push_back(costmap_2d::toPoint32(point->point));
  }
  else if (input_.header.frame_id != point->header.frame_id)
  {
    ROS_ERROR("Frame mismatch, restarting polygon selection");
    input_.polygon.points.clear();
  }
  else if (input_.polygon.points.size() > 1
      && heatmap::pointsNearby(input_.polygon.points.front(), point->point, average_distance * 0.1))
  {
    ROS_INFO("Last point received!");
    //check if last boundary point, i.e. nearby to first point
    if (input_.polygon.points.size() < 3)
    {
      ROS_ERROR("Not a valid polygon, restarting");
    }
    else
      polygon_planner_pub.publish(input_);

    input_.polygon.points.clear();
  }
  else
  {
    input_.polygon.points.push_back(costmap_2d::toPoint32(point->point));
    input_.header.stamp = ros::Time::now();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "points_to_polygon");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("clicked_point", 10, &pointCb);
  point_viz_pub_ = n.advertise<visualization_msgs::Marker>("exploration_polygon_marker", 10);
  polygon_planner_pub = n.advertise<geometry_msgs::PolygonStamped>("field/cut_area", 10);

  point_viz_timer_ = n.createWallTimer(ros::WallDuration(0.1), boost::bind(&vizPubCb));
  ros::spin();

  return 0;
}
