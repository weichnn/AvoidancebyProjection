#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include "pcl_ros/point_cloud.h"
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
ros::Publisher pub;


pcl::PointCloud<pcl::PointXYZ> msg;
int count;
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  
    pcl::PCLPointCloud2 cloud_blob;
    pcl_conversions::toPCL(*input,cloud_blob);
    pcl::fromPCLPointCloud2 (cloud_blob, msg);
    
  

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cloud_source");
  ros::NodeHandle nh;
  count = 0;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("map_output", 10, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_source", 10);

  
  ros::Rate loop_rate(10);
  tf::TransformListener listener;
  while (ros::ok())
  {
     
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
      ros::Time now = ros::Time::now();
      cloud->points.resize (msg.points.size());
      listener.waitForTransform("world", "robot",
                              now, ros::Duration(3.0));
   for (size_t i = 0; i < msg.points.size (); ++i){
       //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
      geometry_msgs::PointStamped laser_point;
      laser_point.header.frame_id = "world";
      laser_point.header.stamp = now;
      laser_point.point.x = msg.points[i].x;
      laser_point.point.y = msg.points[i].y;
      laser_point.point.z = msg.points[i].z;
      geometry_msgs::PointStamped base_point; 
      listener.transformPoint("robot", laser_point, base_point);

      cloud->points[i].x = base_point.point.x;
      cloud->points[i].y = base_point.point.y;
      cloud->points[i].z = base_point.point.z;
   }
  
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud (cloud);
      pass.setFilterFieldName ("x");
      pass.setFilterLimits (0.0, 2.0);
      //pass.setFilterLimitsNegative (true);
      pass.filter (*cloud_filtered);
  
      pcl::PassThrough<pcl::PointXYZ> pass2;
      pass2.setInputCloud (cloud_filtered);
      pass2.setFilterFieldName ("y");
      pass2.setFilterLimits (-1.0, 1.0);
      //pass.setFilterLimitsNegative (true);
      pass2.filter (*cloud_final);
  
      pcl::PassThrough<pcl::PointXYZ> pass3;
      pass3.setInputCloud (cloud_final);
      pass3.setFilterFieldName ("z");
      pass3.setFilterLimits (-1.0, 1.0);
      //pass.setFilterLimitsNegative (true);
      pass3.filter (*cloud_final);
      
      // Create a container for the data.
      sensor_msgs::PointCloud2 output;
    
      // Do data processing here...
      pcl::toROSMsg(*cloud_final,output);
      output.header.frame_id = std::string("robot");
      output.header.stamp = now;
      // Publish the data.
      pub.publish (output);
      count++;
      printf("It's %d\n",count);

      ros::spinOnce();

      loop_rate.sleep();
  }

}
