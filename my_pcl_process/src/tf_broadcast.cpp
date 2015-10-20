#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");

  ros::NodeHandle node;
  ros::Rate loop_rate(50);
  int count = 0;
  while (ros::ok())
  {
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(count*0.001, 0, 0) );
      tf::Quaternion q;
      q.setRPY(0, 0, count*0.01);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot"));

      ros::spinOnce();

      loop_rate.sleep();
      ++count;
  }
  return 0;
};