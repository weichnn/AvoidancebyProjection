#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pcl_ros/point_cloud.h"
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "sensor_msgs/PointCloud2.h"
#include "brushfire.cpp"
#include "math.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <iostream>
#include "Eigen/Dense"
#include "QP.cpp"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include "pcl/octree/octree.h"
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

float LF[] = {0.5,1,1.5,2,3.08,3.1,3.11,2.2,2.6,2.8,3,3.69,4.24,5.13,6.5};
ros::Subscriber sub;
ros::Publisher wayPointPub;
ros::Publisher pathPub;
nav_msgs::Path globalPath;
nav_msgs::Path localPointPath;
nav_msgs::Path localWayPath;
bool isUpdate;
MatrixXd optim_polyX;
MatrixXd optim_polyY;
MatrixXd optim_polyZ;
vector< double >VAX;
vector< double >VAY;
vector< double >VAZ;
MatrixXd time_kfBK;


using namespace Eigen;


int compareIndex(float num,vector<float>LFinter)
{
  int i = 0;
  while(true )
  {
    if(i < LFinter.size()){
      
    
      if(num <= LFinter[i]){
      
	return i;
      }
      else{
	i++;
      }
    
    }
    else{
      return LFinter.size()-1;
    }
  }
}
float att(float factor,Position one,Position two){
    return factor*((one.x-two.x)*(one.x-two.x)+(one.y-two.y)*(one.y-two.y))/2;
};
float rep(float factor,float dist){
    return factor*pow(30/dist,2)/2;
};

void globalPathProcess(const nav_msgs::Path msg)
{
    globalPath = msg;
}

void chatterCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
  vector< vector< vector< int > > >matries;
  vector< vector< Position > >obstalePointsList(14);	
  vector <Position> lastPointList;
  vector< Position> metaData;
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  int halfMaxHeight = 0;
  int halfMaxWidth = 0;
  int maxD = 0;
  bool isSafe = false;
  nav_msgs::Path thePath;
  nav_msgs::Path theWayPath;
  vector< float > theCollision;
  vector< float > Xdist;

  for(int i = 0; i<4 ;i++){
    Xdist.push_back(LF[i]);
  }

	  float resolution = 0.1f;
	  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
	  octree.setInputCloud (msg);
	  octree.addPointsFromInputCloud ();

  ros::Time begin_time = ros::Time::now();
  
  
  while(isSafe != true){

     // for(int i = 0;i<(theCollision.size());i++){
      if(theCollision.size() != 0){
	while((theCollision[0]-0.1)<Xdist[Xdist.size()-1]){
	  Xdist.pop_back();
	}
	Xdist.push_back(theCollision[0]-0.1);
      }
     // }
      theCollision.clear();
    //  for(int i =0 ;i <Xdist.size();i++){
	//printf("%f\n",Xdist[i]);
    //  }
     // sort(Xdist.begin(), Xdist.end()); 
    //printf("it's ok here\n");
      BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){
   
	//printf("%f,%f,%f\n",pt.x,pt.y,pt.z);
	if(!isnan(pt.x)&&!isnan(pt.y)&&!isnan(pt.z)){
	    int i = compareIndex(pt.x,Xdist);
	    int theZ = pt.z*10;
	    int theY = pt.y*10;
	    int z;
	    int y;
	    if(i>maxD){
	      maxD = i;
	    }
	    //printf("%d,%d",maxD,i);
	    if(z!=theZ||y!=theY){
	      z = theZ;
	      y = theY;
	      Position point;
	      point.x = z;
	      point.y = y;
	      if(abs(point.x)>halfMaxWidth){
		halfMaxWidth = abs(point.x);
	      }
	      if(abs(point.y)>halfMaxHeight){
		halfMaxHeight = abs(point.y);
	      }
	//printf ("%d,%d,%d\n",i,point.x,point.y);

	      for(int j = i;j<Xdist.size();j++){
	  //printf("test:");
	      obstalePointsList.at(j).push_back(point);  
	      }
	
	    }
	}
    //printf("it's complete\n");  
      }

    //printf ("%d,%d\n",halfMaxWidth,halfMaxHeight);
    nav_msgs::Path path;
    path.header = std_msgs::Header();
    path.header.frame_id = "robot";
    path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.pose.position.x = 0;
    poseStamped.pose.position.y = 0;
    poseStamped.pose.position.z = 0;
    path.poses.push_back(poseStamped);

    vector <Position> PointList;
    static Position prevPosition = {0,0};
    for(int i = 0; i<maxD+1; i++){
      if(obstalePointsList.at(i).size()!=0){
	vector< vector<int> >theMaries(halfMaxWidth*2+1, vector<int>(halfMaxHeight*2+1, -1));
	Position x = brushfire(theMaries,obstalePointsList.at(i));
	//displayGrid(theMaries);
	
	
	
	  float xx = Xdist[i] - path.poses[i].pose.position.x;
	  float yy = (float)x.y/10 - path.poses[i].pose.position.y;
	  float zz = (float)x.x/10 - path.poses[i].pose.position.z;

	  pcl::PointXYZ searchPoint;
	  float radius = 0.2;
	  std::vector<int> pointIdxRadiusSearch;
	  std::vector<float> pointRadiusSquaredDistance;
	  for(float j = 0.0; j <= 1.0 ; j+=0.1){
	      searchPoint.x = path.poses[i].pose.position.x + j*xx;
	      searchPoint.y = path.poses[i].pose.position.y + j*yy;
	      searchPoint.z = path.poses[i].pose.position.z + j*zz;
	      
	      //printf("%f,%f,%f\n",j*xx,j*yy,j*zz);
	      if (octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	      {
		 // printf("%f,%f,%f\n",searchPoint.x,searchPoint.y,searchPoint.z);
		  theCollision.push_back(searchPoint.x);
		  break;
	      }
	  }
	  if(theCollision.size() != 0){
	    if(searchPoint.x == 0) break;
	    geometry_msgs::PoseStamped poseStamped;
	    poseStamped.pose.position.x = searchPoint.x;
	    poseStamped.pose.position.y = searchPoint.y;
	    poseStamped.pose.position.z = searchPoint.z;
	//printf("%f,%f,%f\n",poseStamped.pose.position.x,poseStamped.pose.position.y,poseStamped.pose.position.z);
	    path.poses.push_back(poseStamped);
	      break;
	  }
	  else{
	    //printf("get:%d,%d\n",x.x,x.y);
	    geometry_msgs::PoseStamped poseStamped;
	    poseStamped.pose.position.x = Xdist[i];
	    poseStamped.pose.position.y = (float)x.y/10;
	    poseStamped.pose.position.z = (float)x.x/10;
	//printf("%f,%f,%f\n",poseStamped.pose.position.x,poseStamped.pose.position.y,poseStamped.pose.position.z);
	    path.poses.push_back(poseStamped);
	//lastPointList.push_back(x);;
	  }
	
     }
    
   }
   if(theCollision.size() == 0){
      isSafe = true;
   }
   isSafe = true;
   thePath = path;
  }
  
  
   if(thePath.poses.size() == 1){
     return;
   }
   
  int num = 0;
  isSafe = false;
  theCollision.clear();
  theCollision.push_back(0);
  while(isSafe != true){
    nav_msgs::Path waypath;
    waypath.header = std_msgs::Header();
    waypath.header.frame_id = "robot";
    waypath.header.stamp = ros::Time::now();
 
    
    MatrixXd listX(1,thePath.poses.size());
    MatrixXd listY(1,thePath.poses.size());
    MatrixXd listZ(1,thePath.poses.size());
    MatrixXd time_kf(1,thePath.poses.size());
    int bc = 0;
    for(int i = 0; i < thePath.poses.size(); i++){
	listX(0,i) = thePath.poses[i].pose.position.x;
	listY(0,i) = thePath.poses[i].pose.position.y;
	listZ(0,i) = thePath.poses[i].pose.position.z;
	time_kf(0,i) = i*2;
      
    }
    time_kfBK = time_kf;
    std::cout<<"X:" << listX <<std::endl<< "Y:" << listY <<std::endl<< "Z:" << listZ <<std::endl;
    
    optim_polyX = QP(listX,time_kf,VAX[0],VAX[1]);
    vector< double >trajX = getTraj(optim_polyX,time_kf);

    
    optim_polyY = QP(listY,time_kf,VAY[0],VAY[1]);
    vector< double >trajY = getTraj(optim_polyY,time_kf);

    
    optim_polyZ = QP(listZ,time_kf,VAZ[0],VAZ[1]);
    vector< double >trajZ = getTraj(optim_polyZ,time_kf);


    pcl::PointXYZ searchPoint;
    float radius = 0.1;
    for (int i = 0; i< trajX.size(); i++) {
        searchPoint.x = trajX[i];
	searchPoint.y = trajY[i];
	searchPoint.z = trajZ[i];
	
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;


	if (octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
	  theCollision.clear();
	  theCollision.push_back(trajX[i]);

	  break;
	}
	
      
    	geometry_msgs::PoseStamped poseStamped;
	poseStamped.pose.position.x = trajX[i];
	poseStamped.pose.position.y = trajY[i];
	poseStamped.pose.position.z = trajZ[i];
	waypath.poses.push_back(poseStamped);
    }
    if(theCollision.size() == 0){
      isSafe = true;
    }
    isSafe = true;
   // num++;
    theWayPath = waypath;

  }
    ros::Time end_time = ros::Time::now();
    ROS_INFO("It use %f",(end_time-begin_time).toSec());
    localPointPath = thePath;
    localWayPath = theWayPath;
    wayPointPub.publish(thePath);
    pathPub.publish(theWayPath);
    isUpdate = true;
  //ROS_INFO("I heard: %d and %f and %d",msg.height,*x,msg.data[0]);
  
  
}





int main(int argc, char **argv)
{

  ros::init(argc, argv, "path_build");
  ros::NodeHandle n;
  ROS_INFO("It's beginning");

  sub = n.subscribe("/cloud_source", 10, chatterCallback);
  ros::Subscriber globalpathsub = n.subscribe("/globe_path", 10, globalPathProcess);
  wayPointPub = n.advertise<nav_msgs::Path>("way_point",10);
  pathPub = n.advertise<nav_msgs::Path>("way_path",10);
  ros::Publisher real_path_pub = n.advertise<nav_msgs::Path>("real_path", 10);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformListener listener;
  nav_msgs::Path real_pathway;
  real_pathway.header.frame_id = "world";
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  double xxxz = 0;
  double xyzz = 0;
  double x = 0.2;
  double y = -6;
  double z = 0.0;
  double thy = 0.0;
  double thz = atan(40.0);
  double t = 0;
  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;
  double vthy = 0.0;
  double vthz = 0.0;
  VAX.push_back(0);
  VAX.push_back(0);
  VAY.push_back(0);
  VAY.push_back(0);
  VAZ.push_back(0);
  VAZ.push_back(0);
  isUpdate = false;
  bool isComplete = false;
  int globalWayPointNum = 0;
  ros::Rate loop_rate(20);
  
  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    
    
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();;	
    odom_trans.header.frame_id = "world";
    odom_trans.child_frame_id = "robot";

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,thz);	
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = z;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);
    
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    if(isUpdate == true){
      t = dt;
      isUpdate = false;
    }
    else{
      t += dt;
    }
    if(globalPath.poses.size()==0 || localWayPath.poses.size() == 0 ||isComplete){
      
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;	
	odom_trans.header.frame_id = "world";
	odom_trans.child_frame_id = "robot";

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,thz);	
	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = z;
	odom_trans.transform.rotation = odom_quat;
	odom_broadcaster.sendTransform(odom_trans);
    
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "world";

	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = z;
	odom.pose.pose.orientation = odom_quat;

	odom.child_frame_id = "robot";

	odom_pub.publish(odom);

    }
    else{
	listener.waitForTransform("/robot", "/world",
                              localWayPath.header.stamp, ros::Duration(3.0));
        geometry_msgs::PointStamped tfpoint;
	tfpoint.header.frame_id = localWayPath.header.frame_id;
	tfpoint.header.stamp = localWayPath.header.stamp;
	double scale = 0.005;
	tfpoint.point.x = localWayPath.poses[(int)(t/scale)].pose.position.x;
	tfpoint.point.y = localWayPath.poses[(int)(t/scale)].pose.position.y;
	tfpoint.point.z = localWayPath.poses[(int)(t/scale)].pose.position.z;
	geometry_msgs::PointStamped afTfPoint;
        listener.transformPoint("world", tfpoint, afTfPoint);
	
        x = afTfPoint.point.x;
	y = afTfPoint.point.y;
	z = afTfPoint.point.z;
	
	for(int i = (int)(t/scale);i<=(int)((t+dt)/scale);i++){
	    geometry_msgs::PointStamped smooth_point;
	    smooth_point.header.frame_id = localWayPath.header.frame_id;
	    smooth_point.header.stamp = localWayPath.header.stamp;
	    smooth_point.point.x = localWayPath.poses[i].pose.position.x;
	    smooth_point.point.y = localWayPath.poses[i].pose.position.y;
	    smooth_point.point.z = localWayPath.poses[i].pose.position.z;
	    geometry_msgs::PointStamped next_smooth_Point;
	    listener.transformPoint("world", smooth_point, next_smooth_Point);
	  
	    geometry_msgs::PoseStamped poseStamped;
	    poseStamped.pose.position.x = next_smooth_Point.point.x;
	    poseStamped.pose.position.y = next_smooth_Point.point.y;
	    poseStamped.pose.position.z = next_smooth_Point.point.z;
	    real_pathway.poses.push_back(poseStamped);
	}
	real_pathway.header.stamp = current_time;
	real_path_pub.publish(real_pathway);
	
	geometry_msgs::PointStamped nexttfpoint;
	nexttfpoint.header.frame_id = localWayPath.header.frame_id;
	nexttfpoint.header.stamp = localWayPath.header.stamp;
	nexttfpoint.point.x = localWayPath.poses[(int)((t+dt)/scale)].pose.position.x;
	nexttfpoint.point.y = localWayPath.poses[(int)((t+dt)/scale)].pose.position.y;
	nexttfpoint.point.z = localWayPath.poses[(int)((t+dt)/scale)].pose.position.z;
	geometry_msgs::PointStamped nextafTfPoint;
        listener.transformPoint("world", nexttfpoint, nextafTfPoint);
	
	
	VAX = getVandA(optim_polyX,time_kfBK,t);	
	VAY = getVandA(optim_polyY,time_kfBK,t);
	VAZ = getVandA(optim_polyZ,time_kfBK,t);
	geometry_msgs::Vector3Stamped vectorAcc;
	vectorAcc.header.frame_id = localWayPath.header.frame_id;
	vectorAcc.header.stamp = localWayPath.header.stamp;
	vectorAcc.vector.x = VAX[1];
	vectorAcc.vector.y = VAY[1];
	vectorAcc.vector.z = VAZ[1];
	geometry_msgs::Vector3Stamped vectorAccAns;
	geometry_msgs::Vector3Stamped vectorVco;
	vectorVco.header.frame_id = localWayPath.header.frame_id;
	vectorVco.header.stamp = localWayPath.header.stamp;
	vectorVco.vector.x = VAX[0];
	vectorVco.vector.y = VAY[0];
	vectorVco.vector.z = VAZ[0];
	geometry_msgs::Vector3Stamped vectorVcoAns;
	listener.transformVector("world",vectorAcc,vectorAccAns);
	listener.transformVector("world",vectorVco,vectorVcoAns);
	//printf("%f\n",atan2((nextafTfPoint.point.y-y),(nextafTfPoint.point.x-x))/3.14*180);
        //thy = atan2((nextafTfPoint.point.z-z),sqrt((nextafTfPoint.point.x-x)*(nextafTfPoint.point.x-x)+(nextafTfPoint.point.z-z)*(nextafTfPoint.point.z-z)));
	
	//printf("after:%f,%f\n",thy,thz);
	//if(nextafTfPoint.point.y != y||nextafTfPoint.point.x!=x){
	//  thz = atan2((nextafTfPoint.point.y-y),(nextafTfPoint.point.x-x));
	//}
	while((x-globalPath.poses[globalWayPointNum].pose.position.x)*(x-globalPath.poses[globalWayPointNum].pose.position.x)
	  +(y-globalPath.poses[globalWayPointNum].pose.position.y)*(y-globalPath.poses[globalWayPointNum].pose.position.y)
	  +(z-globalPath.poses[globalWayPointNum].pose.position.z)*(z-globalPath.poses[globalWayPointNum].pose.position.z)
	  < 1){
	  globalWayPointNum++;
	  if(globalPath.poses.size()==globalWayPointNum){
	    globalWayPointNum--;
	    if((x-globalPath.poses[globalWayPointNum].pose.position.x)*(x-globalPath.poses[globalWayPointNum].pose.position.x)
	  +(y-globalPath.poses[globalWayPointNum].pose.position.y)*(y-globalPath.poses[globalWayPointNum].pose.position.y)
	  +(z-globalPath.poses[globalWayPointNum].pose.position.z)*(z-globalPath.poses[globalWayPointNum].pose.position.z)
	  < 0.3)
	    isComplete = true;
	    break;
	  }
	}
	//thz = atan2((globalPath.poses[globalWayPointNum].pose.position.y-y),(globalPath.poses[globalWayPointNum].pose.position.x-x));
	
	thz = atan2((globalPath.poses[globalWayPointNum].pose.position.y-globalPath.poses[globalWayPointNum-1].pose.position.y),(globalPath.poses[globalWayPointNum].pose.position.x-globalPath.poses[globalWayPointNum-1].pose.position.x));
	
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;	
	odom_trans.header.frame_id = "world";
	odom_trans.child_frame_id = "robot";

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,thz);	
	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = z;
	odom_trans.transform.rotation = odom_quat;
	odom_broadcaster.sendTransform(odom_trans);
	
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "world";
	
	if(nextafTfPoint.point.y != y||nextafTfPoint.point.x!=x){
	   xxxz = atan2((nextafTfPoint.point.y-y),(nextafTfPoint.point.x-x));
	   xyzz =  -1*atan2((nextafTfPoint.point.z-z),sqrt((nextafTfPoint.point.y-y)*(nextafTfPoint.point.y-y)+(nextafTfPoint.point.x-x)*(nextafTfPoint.point.x-x)));
	}
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = z;
	odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,xyzz,xxxz);;
	
	odom.child_frame_id = "robot";
	odom.twist.twist.linear.x = vectorVco.vector.x;
	odom.twist.twist.linear.y = vectorVco.vector.y;
	odom.twist.twist.linear.z = vectorVco.vector.z;
	//odom.twist.twist.angular.y = atan2((nextafTfPoint.point.z-z),(nextafTfPoint.point.x-x))/dt;
	//odom.twist.twist.angular.z = atan2((nextafTfPoint.point.y-y),(nextafTfPoint.point.x-x))/dt;
	
        odom_pub.publish(odom);
	
	//double ang = atan2(VAY[0],VAX[0])-atan2(VAY[1],VAX[1]);
	//double val = sqrt( VAX[1]*VAX[1] + VAY[1]*VAY[1]);
	//VAX[0] = sqrt( VAX[0]*VAX[0] + VAY[0]*VAY[0]);
	//VAX[1] = val*sin(ang);
	//VAY[0] = 0;
	//VAY[1] = val*sin(ang);
	printf("%f,%f,thz:=%f\n",vectorVcoAns.vector.x,vectorVcoAns.vector.y,thz);
	double ang0 = atan2(vectorVcoAns.vector.y,vectorVcoAns.vector.x)- thz;
	double ang1 = atan2(vectorAccAns.vector.y,vectorAccAns.vector.x) - thz;
	double valA0 = sqrt( vectorVcoAns.vector.y*vectorVcoAns.vector.y + vectorVcoAns.vector.x*vectorVcoAns.vector.x);
	double valA1 = sqrt( vectorAccAns.vector.y*vectorAccAns.vector.y + vectorAccAns.vector.x*vectorAccAns.vector.x);
	printf("%f,%f,%f,%f\n",ang0,ang1,valA0,valA1);
	VAX[0] = valA0*cos(ang0);
	VAX[1] = valA1*cos(ang1);
	VAY[0] = valA0*sin(ang0);
	VAY[1] = valA1*sin(ang1);
	printf("%f,%f\n",VAX[0],VAY[0]);
    }
    
    
    
    last_time = current_time;
    loop_rate.sleep();
    ++count;
  }

  return 0;
}