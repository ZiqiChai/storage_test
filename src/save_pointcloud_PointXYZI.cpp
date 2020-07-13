//C++
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

//Customed
#include "mkdir.hpp"

using namespace std;

std::string pkg_loc = ros::package::getPath("storage_test");

void pointcloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& pc)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(*pc, *cloud);
	ros::Time time = ros::Time::now();
	std::stringstream ss;
	ss<<time;
	pcl::io::savePCDFileASCII(pkg_loc+"/data/"+ss.str()+"_ASCII.pcd", *cloud);
	pcl::io::savePCDFileBinary(pkg_loc+"/data/"+ss.str()+"_Binary.pcd", *cloud);
	pcl::io::savePLYFileASCII(pkg_loc+"/data/"+ss.str()+"_ASCII.ply", *cloud);
	pcl::io::savePLYFileBinary(pkg_loc+"/data/"+ss.str()+"_Binary.ply", *cloud);
}

int main(int argc,char **argv)
{
	createDirectory(pkg_loc + "/data/");
	ros::init(argc,argv,"save_pointcloud_PointXYZI");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, pointcloudCallback);
	ros::spinOnce();
	return 1;
}