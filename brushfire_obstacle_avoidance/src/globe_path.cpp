#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher wayPointPub = n.advertise<nav_msgs::Path>("globe_path",10);
  ros::Rate loop_rate(10);
  nav_msgs::Path waypath;
  waypath.header = std_msgs::Header();
  waypath.header.frame_id = "world";

  float x[] = {0.2, 0.1, 0,  1, 5,  6};
  float y[] = { -6,  -2, -1,0.5, 1,0.9};
  float z[] = {0,0,0,0,0,0};
  for(int i = 0; i< 6 ;i++){
    geometry_msgs::PoseStamped poseStamped1;
    poseStamped1.pose.position.x = x[i];
    poseStamped1.pose.position.y = y[i];
    poseStamped1.pose.position.z = z[i];
   
    waypath.poses.push_back(poseStamped1);

  }
  
  
  int count = 0;
  while (ros::ok())
  {
    waypath.header.stamp = ros::Time::now();
    wayPointPub.publish(waypath);


    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}