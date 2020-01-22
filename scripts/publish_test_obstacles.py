#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32


def publish_obstacle_msg():
  #pub = rospy.Publisher('/test_optim_node/obstacles', ObstacleArrayMsg, queue_size=1)
  pub = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
  rospy.init_node("test_obstacle_msg")


  obstacle_msg = ObstacleArrayMsg() 
  obstacle_msg.header.stamp = rospy.Time.now()
  obstacle_msg.header.frame_id = "map" # CHANGE HERE: odom/map
  
  # Add point obstacle
  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[0].id = 0
  obstacle_msg.obstacles[0].polygon.points = [Point32()]
  obstacle_msg.obstacles[0].polygon.points[0].x = 3
  obstacle_msg.obstacles[0].polygon.points[0].y = 3
  obstacle_msg.obstacles[0].polygon.points[0].z = 0

  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[1].id = 1
  obstacle_msg.obstacles[1].polygon.points = [Point32()]
  obstacle_msg.obstacles[1].polygon.points[0].x = 3
  obstacle_msg.obstacles[1].polygon.points[0].y = 3
  obstacle_msg.obstacles[1].polygon.points[0].z = 0

  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[2].id = 2
  obstacle_msg.obstacles[2].polygon.points = [Point32()]
  obstacle_msg.obstacles[2].polygon.points[0].x = 3
  obstacle_msg.obstacles[2].polygon.points[0].y = 3
  obstacle_msg.obstacles[2].polygon.points[0].z = 0

  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[3].id = 3
  obstacle_msg.obstacles[3].polygon.points = [Point32()]
  obstacle_msg.obstacles[3].polygon.points[0].x = 3
  obstacle_msg.obstacles[3].polygon.points[0].y = 6
  obstacle_msg.obstacles[3].polygon.points[0].z = 0

  r = rospy.Rate(10) # 10hz
  t = 0.0
  while not rospy.is_shutdown():
    
    # Vary y component of the point obstacle
    
    pub.publish(obstacle_msg)
    
    r.sleep()



if __name__ == '__main__': 
  try:
    publish_obstacle_msg()
  except rospy.ROSInterruptException:
    pass

