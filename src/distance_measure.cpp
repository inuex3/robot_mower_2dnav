/* Copyright 2015 Institute of Digital Communication Systems - Ruhr-University Bochum
 * Author: Adrian Bauer
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
 **/

#include <ros/ros.h>
#include <heatmap/wifi_signal.h>
#include <heatmap/geometry_tools.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <heatmap/interpolation_service.h>
#include <heatmap/signal_service.h>
#include <geometry_msgs/PolygonStamped.h>

#include <vector>
#include <math.h>

namespace heatmap
{
/**
* @brief Node that requests WIFI signals based on the distance traveled by the robot, visualises them via RViz
* and provides the possibility to run an interpolation to get a dense map from it
*/
class DistanceMeasure
{
private:
  heatmap::wifi_signal current_sig;
  ros::Publisher marker_pub, interpol_marker_pub;
  ros::Subscriber signal_sub;
  int sig_count, av_sig;
  geometry_msgs::PolygonStamped polygon;
  std::vector<tf::Vector3> measure_list;
  std::vector<int> data_list;
  int marker_id;
  ros::NodeHandle n;
  int quality_max;

  /**
  * @brief Converts a signal quality value to a color value used for RViz markers
  * @param signal Signal to convert
  * @return Value between 0 and 1 representing the signal quality, 0 means really bad, 1 means really good
  */
  double convertSignalToColor(heatmap::wifi_signal& signal)
  {
    double color;

    color = (((double)signal.link_quality - 35.0) * 2.0) / (double)signal.link_quality_max;
    if (color < 0)
      color = 0;

    return color;
  }

  /**
  * @brief Creates a sphere RViz marker for publishing
  * @param color_g The color for the marker, in range of 0 to 1, 0 is pure red, 1 is pure green
  * @param sphere_size The radius of the sphere
  * @param pos The position of the marker on the map
  * @return A marker
  */
  visualization_msgs::Marker prepareSphereMarker(double color_g, double sphere_size, tf::Vector3& pos)
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "wifi_heatmap";
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pos.x();
    marker.pose.position.y = pos.y();
    marker.pose.position.z = pos.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker
    marker.scale.x = sphere_size;
    marker.scale.y = sphere_size;
    marker.scale.z = sphere_size;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0 - color_g;
    marker.color.g = color_g;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    return marker;
  }

  /**
  * @brief Processes a list of measuring data, interpolates it and visualizes it in RViz
  * @param measure_points List of measuring points (x, y coordinates)
  * @param measure_data List of measuring data according to measure_points
  * @param poly Polygon specifying the measuring area
  * @param interpolation_spacing The space between the interpolated points
  * @param shepard_power Determines how strong the distance between points is weighted. For more information read the wikipedia article
  * @return List of interpolated data
  */
  std::vector<double> interpolateAndVisualize(std::vector<geometry_msgs::Point32>& measure_points,
                                              std::vector<int>& measure_data, geometry_msgs::Polygon& poly,
                                              double interpolation_spacing, double shepard_power)
  {
    std::vector<geometry_msgs::Point32> interpolation_points;
    std::vector<double> interpolation_data;
    std::vector<visualization_msgs::Marker> marker_list;
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;
    heatmap::wifi_signal interpol_signal;
    tf::Vector3 marker_pos;

    /* Fill the measure area polygon with points considering the spacing between them */
    interpolation_points = heatmap::fillPolygon<geometry_msgs::Point32>(poly, interpolation_spacing);
    /* Now interpolate at these points */
    interpolation_data = heatmap::shepardInterpolation(interpolation_points, measure_points, measure_data, shepard_power);

    interpol_signal.link_quality_max = quality_max;

    /* Iterate over the interpolated data and prepare a RViz marker arary for them */
    for (int i = 0; i < interpolation_data.size(); i++)
    {
      interpol_signal.link_quality = interpolation_data.at(i);
      marker_pos.setValue(interpolation_points.at(i).x, interpolation_points.at(i).y, 0);
      marker = prepareSphereMarker(convertSignalToColor(interpol_signal), interpolation_spacing, marker_pos);
      marker_list.push_back(marker);
    }

    /* Publish this marker array */
    marker_array.markers = marker_list;
    interpol_marker_pub.publish(marker_array);
    ROS_INFO("Interpolation done!");

    return interpolation_data;
  }

  bool startInterpolation(heatmap::interpolation_service::Request &req, heatmap::interpolation_service::Response &res)
  {
    std::vector<geometry_msgs::Point32> point_measure_list;
    geometry_msgs::Point32 p;
    /* Convert tf::Vector3 to std::vector */
    for (int i = 0; i < measure_list.size(); i++)
    {
      tf::Vector3 vect = measure_list.at(i);
      p.x = vect.getX();
      p.y = vect.getY();
      point_measure_list.push_back(p);
    }

    ROS_INFO("Interpolation called with spacing: %f and Shepard power: %f", (double )req.spacing,
             (double )req.shepard_power);

    std::vector<double> interpolated = interpolateAndVisualize(point_measure_list, data_list, polygon.polygon,
                                                               (double)req.spacing, (double)req.shepard_power);

    return true;
  }

  void polygonCallback(const geometry_msgs::PolygonStamped& poly)
  {
    polygon = poly;
  }


public:
  DistanceMeasure() :
    n()
{
    sig_count = 0;
    av_sig = 0;
    marker_id = 0;
    /* Distance in meters to take a new measurement */
    const double MEASURE_DISTANCE = 0.3;
    heatmap::signal_service srv;
    heatmap::wifi_signal signal;
    tf::Vector3 last_measure(0, 0, 0), last_transform;
    tfScalar dist;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    visualization_msgs::Marker mark;

    ros::Rate rate(5.0);
    ros::Subscriber polygon_sub = n.subscribe("heatmap_area", 1, &DistanceMeasure::polygonCallback, this);

    marker_pub = n.advertise<visualization_msgs::Marker>("signal_marker", 0);
    interpol_marker_pub = n.advertise<visualization_msgs::MarkerArray>("interpol_marker", 0);

    /* Register a service client to get the signal strength from the signal node */
    ros::ServiceClient client = n.serviceClient<heatmap::signal_service>("get_wifi_signal");
    /* Register a service server to offer the interpolation feature */
    ros::ServiceServer service = n.advertiseService("start_interpol", &DistanceMeasure::startInterpolation, this);
    /* Loop until we receive a shutdown request */
    while (n.ok())
    {
      /* Starting condition: Let's take a new measurement
       * Might be set to false later inside the loop
       */
      bool measure = true;
      try
      {
        /* Wait for a new transform from /map to base_footprint, then look it up */
        // ROS_INFO("Waiting for a transform from /map to base_footprint...");
        listener.waitForTransform("/map", "base_footprint", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/map", "base_footprint", ros::Time(0), transform);
        last_measure = transform.getOrigin();

        /* Iterate over each measurement vector */
        for (std::vector<tf::Vector3>::iterator it = measure_list.begin(); it != measure_list.end(); ++it)
        {
          /* Get the distance from the current transform (position) to the vector */
          dist = last_measure.distance(*it);
          /* If any distance exists which is lower than our limit
           * Don't measure this time and leave the iteration
           */
          if (dist < MEASURE_DISTANCE)
          {
            measure = false;
            break;
          }
        }

        if (measure)
        {
          /* Get the WIFI signal quality via service call */
          if (client.call(srv))
          {
            signal = srv.response.signal;
            /* save the maximum quality. It shouldn't change. */
            quality_max = signal.link_quality_max;
            ROS_INFO("Distance for a new measurement reached at: x=%f, y=%f", last_measure.getX(), last_measure.getY());
            /* Prepare and publish a RViz marker to display the measured WIFI signal */
            mark = prepareSphereMarker(convertSignalToColor(signal), MEASURE_DISTANCE, last_measure);
            marker_pub.publish(mark);
            /* Save the coordinates and corresponding signal quality for interpolation */
            measure_list.push_back(last_measure);
            data_list.push_back(signal.link_quality);
          }
          else
          {
            ROS_ERROR("get_wifi_signal service call failed!");
          }
        }
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      ros::spinOnce();
      rate.sleep();
    }
}
};

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "distance_measure");

  heatmap::DistanceMeasure dm;

  return 0;
}
