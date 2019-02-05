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
 * This file is an extended version of the original file taken from the frontier_exploration ROS package by Paul Bovbel
 **/


#ifndef GEOMETRY_TOOLS_H_
#define GEOMETRY_TOOLS_H_

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point.h>
#include <costmap_2d/costmap_2d.h>

namespace heatmap
{
  /**
  * @brief Calculate distance between two points
  * @param one Point one
  * @param two Point two
  * @return Distance between two points
  */
  template<typename T, typename S>
  double pointsDistance(const T &one, const S &two){
      return sqrt(pow(one.x-two.x,2.0) + pow(one.y-two.y,2.0) + pow(one.z-two.z,2.0));
  }

  /**
  * @brief Calculate polygon perimeter
  * @param polygon Polygon to process
  * @return Perimeter of polygon
  */
  double polygonPerimeter(const geometry_msgs::Polygon &polygon){

      double perimeter = 0;
      if(polygon.points.size()   > 1)
      {
        for (int i = 0, j = polygon.points.size() - 1; i < polygon.points.size(); j = i++)
        {
          perimeter += pointsDistance(polygon.points[i], polygon.points[j]);
        }
      }
      return perimeter;
  }

/**
* @brief Evaluate whether two points are approximately adjacent, within a specified proximity distance.
* @param one Point one
* @param two Point two
* @param proximity Proximity distance
* @return True if approximately adjacent, false otherwise
*/
  template<typename T, typename S>
  bool pointsNearby(const T &one, const S &two, const double &proximity){
      return pointsDistance(one, two) <= proximity;
  }

/**
* @brief Evaluate if point is inside area defined by polygon. Undefined behaviour for points on line.
* @param point Point to test
* @param polygon Polygon to test
* @return True if point is inside polygon, false otherwise
*/
  template<typename T>
  bool pointInPolygon(const T &point, const geometry_msgs::Polygon &polygon){
      int cross = 0;
      for (int i = 0, j = polygon.points.size()-1; i < polygon.points.size(); j = i++) {
          if ( ((polygon.points[i].y > point.y) != (polygon.points[j].y>point.y)) &&
              (point.x < (polygon.points[j].x-polygon.points[i].x) * (point.y-polygon.points[i].y) / (polygon.points[j].y-polygon.points[i].y) + polygon.points[i].x) ){
              cross++;
          }
      }
      return bool(cross % 2);
  }

/**
* @brief Calculate the yaw of vector defined by origin and end points
* @param origin Origin point
* @param end End point
* @return Yaw angle of vector
*/
  template<typename T, typename S>
  double yawOfVector(const T &origin, const S &end){

      double delta_x, delta_y;
      delta_x = end.x - origin.x;
      delta_y = end.y - origin.y;

      double yaw = atan(delta_x/delta_y);

      if(delta_x < 0){
          yaw = M_PI-yaw;
      }

      return yaw;
  }

/**
 * Extended function
 * @brief Calculate the point with minimal x, y coordinates of all polygon points
 * @param polygon Polygon to process
 * @return Point with minimum x, y coordinates
*/
  template<typename T>
  T bottomLeftPointPolygon(const geometry_msgs::Polygon &polygon)
  {
    T bottom_left;

    if(polygon.points.size() > 0)
    {
      bottom_left = polygon.points.front();

      for(int i = 0; i < polygon.points.size(); i++)
      {
        if(polygon.points.at(i).x < bottom_left.x)
          bottom_left.x = polygon.points.at(i).x;
        if(polygon.points.at(i).y < bottom_left.y)
          bottom_left.y = polygon.points.at(i).y;
      }
    }

    return bottom_left;
  }

  /**
   * Extended function
   * @brief Calculate the point with maximal x, y coordinates of all polygon points
   * @param polygon Polygon to process
   * @return Point with maximum x, y coordinates
  */

  template<typename T>
  T topRightPointPolygon(const geometry_msgs::Polygon &polygon)
  {
    T top_right;

    if(polygon.points.size() > 0)
    {
      top_right = polygon.points.front();

      for(int i = 0; i < polygon.points.size(); i++)
      {
        if(polygon.points.at(i).x > top_right.x)
          top_right.x = polygon.points.at(i).x;
        if(polygon.points.at(i).y > top_right.y)
          top_right.y = polygon.points.at(i).y;
      }
    }

    return top_right;
  }

  /**
   * Extended function
   * @brief Fill a polygon with points known the spacing between them
   * @param polygon Polygon to process
   * @param spacing Spacing between points
   * @return Vector of points
  */
  template<typename T>
  std::vector<T> fillPolygon(const geometry_msgs::Polygon &polygon, const double spacing)
  {
    std::vector<T> points_list;
    T bottom_left = bottomLeftPointPolygon<T>(polygon);
    T top_right = topRightPointPolygon<T>(polygon);

    T test_point = bottom_left;

      while (test_point.y <= top_right.y)
      {
        while (test_point.x <= top_right.x)
        {
          if (heatmap::pointInPolygon(test_point, polygon))
            points_list.push_back(test_point);

          test_point.x += spacing;
        }
        test_point.y += spacing;
        test_point.x = bottom_left.x;
      }
    return points_list;
  }

  /**
   * Extended function
   * @brief Interpolate a known scattered data set using the Shepard (aka inverse distance weighting) interpolation
   * @param interpolation_points Vector of points (coordinates) to interpolate the data at
   * @param data_points Vector of points (coordinates) of the input data
   * @param data Vector of data entries corresponding to the coordinates in data_points
   * @param shepard_power Determines how strong the distance between points is weighted. For more information read the wikipedia article
   * @return Vector of interpolated data corresponding to the coordinates in interpolation_points
  */
  template<typename T, typename S>
  std::vector<double> shepardInterpolation(const std::vector<T> interpolation_points, const std::vector<T> data_points, const std::vector<S> data, const double shepard_power)
  {
    std::vector<double> interpolated_points;
    long double lambda_sum = 0, other_sum = 0;
    double interpolation = 0;

    for(int i = 0; i < interpolation_points.size(); i++)
    {
      T p = interpolation_points.at(i);
      for(int j = 0; j < data_points.size(); j++)
      {
        T q = data_points.at(j);
        double dist = heatmap::pointsDistance(p, q);
        long double d = 1.0 / (pow(dist, shepard_power));
        lambda_sum += d;
        other_sum += d * data.at(j);
      }
      interpolation = (double)(other_sum / lambda_sum);
      interpolated_points.push_back(interpolation);

      lambda_sum = other_sum = 0;
    }
    return interpolated_points;
  }
}
#endif
