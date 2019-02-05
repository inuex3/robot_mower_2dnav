/* Original work Copyright Paul Bovbel
 * Modified work Copyright 2015 Institute of Digital Communication Systems - Ruhr-University Bochum
 * Modified by: Adrian Bauer
 *
 * This program is free software; you can redistribute it and/or modify it under the terms of
 * the GNU General Public License as published by the Free Software Foundation;
 * either version 3 of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with this program;
 * if not, see <http://www.gnu.org/licenses/>.
 *
 * This file is a modified version of the original file taken from the frontier_exploration ROS package by Paul Bovbel
 **/

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <heatmap/geometry_tools.h>

#include <ros/wall_timer.h>

#include <visualization_msgs/Marker.h>

namespace heatmap
{

/**
 * @brief Client for FrontierExplorationServer that receives control points from rviz, and creates boundary polygon for frontier exploration
 */
class HeatmapClient
{

private:

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber point_;
  ros::Publisher point_viz_pub_, polygon_planner_pub;
  ros::WallTimer point_viz_timer_;
  geometry_msgs::PolygonStamped input_;

  /**
   * @brief Publish markers for visualization of points for boundary polygon.
   */
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

      for(int i = 0; i < input_.polygon.points.size(); i ++)
      {
        geometry_msgs::Point32 point = input_.polygon.points.at(i);
        line_strip.points.push_back(costmap_2d::toPoint(point));
        points.points.push_back(costmap_2d::toPoint(point));
      }

    points.color.a = points.color.b = line_strip.color.b = line_strip.color.a = 1.0;

    }
    else
    {
      points.action = line_strip.action = visualization_msgs::Marker::DELETE;
    }

    point_viz_pub_.publish(points);
    point_viz_pub_.publish(line_strip);
  }

  /**
   * @brief Build boundary polygon from points received through rviz gui.
   * @param point Received point from rviz
   */
  void pointCb(const geometry_msgs::PointStampedConstPtr& point)
  {

    double average_distance = polygonPerimeter(input_.polygon) / input_.polygon.points.size();

    if (input_.polygon.points.empty())
    {
      //first control point, so initialize header of boundary polygon

      input_.header = point->header;
      input_.polygon.points.push_back(costmap_2d::toPoint32(point->point));

    }
    else if (input_.header.frame_id != point->header.frame_id)
    {
      ROS_ERROR("Frame mismatch, restarting polygon selection");
      input_.polygon.points.clear();

    }
    else if (input_.polygon.points.size() > 1
        && pointsNearby(input_.polygon.points.front(), point->point, average_distance * 0.1))
    {
      //check if last boundary point, i.e. nearby to first point

      if (input_.polygon.points.size() < 3)
        ROS_ERROR("Not a valid polygon, restarting");
      else
        polygon_planner_pub.publish(input_);

      input_.polygon.points.clear();
    }
    else
    {
      //otherwise, must be a regular point inside boundary polygon
      input_.polygon.points.push_back(costmap_2d::toPoint32(point->point));
      input_.header.stamp = ros::Time::now();
    }

  }

public:
  /**
   * @brief Constructor for the client.
   */
  HeatmapClient() :
      nh_(), private_nh_("~")
  {
    input_.header.frame_id = "map";
    point_ = nh_.subscribe("/clicked_point", 10, &HeatmapClient::pointCb, this);
    polygon_planner_pub = nh_.advertise<geometry_msgs::PolygonStamped>("heatmap_area", 10);
    point_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("heatmap_polygon_marker", 10);
    point_viz_timer_ = nh_.createWallTimer(ros::WallDuration(0.1),
                                           boost::bind(&HeatmapClient::vizPubCb, this));
    ROS_INFO("Please use the 'Point' tool in Rviz to select a heatmap boundary.");
  }

};

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explore_client");

  heatmap::HeatmapClient client;
  ros::spin();
  return 0;
}
