#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
ros::Publisher pub;

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("map_output", 10);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_x (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xy (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/weic/catkin_ws/src/my_pcl_process/nsh_2nd_floor_full_zup.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";
 
  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

  sensor_msgs::PointCloud2 output;
  
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-5.0, 5.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_x);
  
  pcl::PassThrough<pcl::PointXYZ> pass2;
  pass2.setInputCloud (cloud_x);
  pass2.setFilterFieldName ("y");
  pass2.setFilterLimits (-5.0, 5.0);
  //pass.setFilterLimitsNegative (true);
  pass2.filter (*cloud_xy);
  
  pcl::toROSMsg(*cloud_filtered,output);
 // pcl_conversions::fromPCL(*cloud_xy, output);
  
  output.header.frame_id = std::string("world");
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
      pub.publish (output);

      ros::spinOnce();

      loop_rate.sleep();
      ++count;
  }
  
  

}