//C++
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

//OpenCV
#include <opencv2/opencv.hpp>

#define CV_WAITKEY_SPACE 32
#define CV_WAITKEY_ESC 27
#define CV_WAITKEY_TAB 9
#define CV_WAITKEY_ENTER 13

using namespace std;

std::string pkg_loc = ros::package::getPath("storage_test");

int saved_count = 0;

void imageCallback(const sensor_msgs::ImageConstPtr& colormsg)
{
	cv::Mat color;
	color = cv_bridge::toCvShare(colormsg,"bgr8")->image.clone();
	imshow("rgb-image", color);
	if (cv::waitKey(20) == CV_WAITKEY_SPACE)
	{
		std::stringstream ss;
		ss << saved_count;
		std::string filename = pkg_loc+"/data/RGB_image_"+ss.str()+".png";
		cv::imwrite(filename, color);
		std::cout << "image " << filename << " saved successfully!" << std::endl;
		saved_count++;
	}
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"rgb_collector");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe(argv[1], 1, imageCallback);
	ros::spin();
	return 1;
}