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
#include <heatmap/signal_service.h>

namespace heatmap
{
/**
* @brief Class that simulates a WIFI network with randomly generated signal qualities.
* It advertises a topic and a service for that.
*/
class SignalSimulator
{
private:
  heatmap::wifi_signal ws;
  ros::NodeHandle nh;

  bool srvGetSignal(heatmap::signal_service::Request &req, heatmap::signal_service::Response &res)
  {
    res.signal = ws;

    return true;
  }
public:
  SignalSimulator() : nh()
{
    ros::Publisher signal_pub = nh.advertise<heatmap::wifi_signal>("signal", 1000);
    ros::ServiceServer service = nh.advertiseService("get_wifi_signal", &SignalSimulator::srvGetSignal, this);

    ros::Rate loop_rate(10);

    ws.essid = "Sim WIFI";
    ws.link_quality_max = 70;

    while(nh.ok())
    {
      ws.link_quality = rand() % 71;
      signal_pub.publish(ws);

      ros::spinOnce();
      loop_rate.sleep();
    }
}
};
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "signal_meter_sim");

  heatmap::SignalSimulator sig_sim;

  return 0;
}
