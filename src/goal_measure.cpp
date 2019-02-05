#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <string>

#include <heatmap/wifi_signal.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient* ac;
ros::Publisher marker_pub;
uint32_t shape = visualization_msgs::Marker::SPHERE;
int marker_index = 0;
heatmap::wifi_signal current_sig;
std::string tf_prefix;

int sig_count = 0, av_sig = 0;

void goalCallback(const geometry_msgs::PoseStamped& lgoal)
{
  move_base_msgs::MoveBaseGoal goal;
  visualization_msgs::Marker marker;
  double link;

  goal.target_pose = lgoal;
  ROS_INFO("I heard: A goal!");
  ROS_INFO("Redirecting goal...");
  ac->sendGoal(goal);

  ac->waitForResult();

  if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, the base reached its goal!");
    ROS_INFO("Let's take a measurement...");
    // Get Wifi signal strength here...
    ROS_INFO("Done!");
    marker.header.frame_id = tf_prefix + "/base_footprint";
    marker.header.stamp = ros::Time::now();
    marker.ns = "wifi_signal";
    marker.id = marker_index;
    marker_index++;
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    link = ((double)((current_sig.link_quality - 35.0)* 2.0)/(double)current_sig.link_quality_max);
    if (link < 0) link = 0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0 - link;
    marker.color.g = link;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    marker_pub.publish(marker);
  }
  else
  {
    ROS_INFO("The base failed to reach its goal for some reason.");
  }
}

void signalCallback(const heatmap::wifi_signal& sig)
{
  av_sig += sig.link_quality;
  sig_count++;

  if(sig_count == 5) {
    current_sig = sig;
    current_sig.link_quality = av_sig/5;
    av_sig = 0;
    sig_count = 0;
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  ros::NodeHandle n;

  n.getParam("/tf_prefix", tf_prefix);
  ROS_INFO("tf_prefix: %s", tf_prefix.c_str());

  ros::Subscriber goal_sub = n.subscribe("goal", 1, goalCallback);
  ros::Subscriber signal_sub = n.subscribe("signal", 1, signalCallback);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  //tell the action client that we want to spin a thread by default
  ac = new MoveBaseClient("move_base", true);

  //wait for the action server to come up
  while(!ac->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ros::spin();
  
  delete ac;
  return 0;
}
