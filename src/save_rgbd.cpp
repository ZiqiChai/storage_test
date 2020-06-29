//C++
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//OpenCV
#include <opencv2/opencv.hpp>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;

std::string pkg_loc = ros::package::getPath("storage_test");

boost::shared_ptr<visualization::CloudViewer> viewer;

unsigned int filesNum = 0;
unsigned int filesNum_ = 0;

bool saveCloud(false);
bool saveImg(false);
bool saveDepth(false);


void cloudCB(const sensor_msgs::PointCloud2& input)
{
    pcl::PointCloud<pcl::PointXYZ> cloud; // With color

    pcl::fromROSMsg(input, cloud); // sensor_msgs::PointCloud2 ----> pcl::PointCloud<T>

    if(! viewer->wasStopped()) viewer->showCloud(cloud.makeShared());

    if(saveCloud)
    {
        stringstream stream;
        stream << pkg_loc << "/data/inputCloud"<< filesNum<< ".ply";
        string filename = stream.str();

        if(io::savePLYFile(filename, cloud, true) == 0)
        {
            filesNum++;
            cout << filename<<" Saved."<<endl;
        }
        else PCL_ERROR("Problem saving %s.\n", filename.c_str());

        saveCloud = false;
    }
}


void imageCallbackRGB(const sensor_msgs::ImageConstPtr &original_image)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr=cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Not able to convert sensor_msgs::Image to OpenCV::Mat format %s", e.what());
        return;
    }

    cv::Mat src_image = cv_ptr->image.clone();

    cv::imshow("rgb", src_image);
    cv::waitKey(20);

    if(saveImg)
    {
        stringstream stream;
        stream << pkg_loc << "/data/inputRGB"<< filesNum_<< ".png";
        string filename = stream.str();

        if(cv::imwrite(filename, src_image))
        {
            cout << filename<<" Saved."<<endl;
        }

        saveImg = false;
    }
}


void imageCallbackDepth(const sensor_msgs::ImageConstPtr &original_image)
{
   cv::Mat depth;
   cv_bridge::CvImageConstPtr pCvImage;// 声明一个CvImage指针的实例
   pCvImage = cv_bridge::toCvShare(original_image, original_image->encoding);//将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
   pCvImage->image.copyTo(depth);

   cv::imshow("depth", depth);
   cv::waitKey(20);
    if(saveDepth)
    {
        stringstream stream;
        stream << pkg_loc << "/data/inputDepth"<< filesNum_<< ".png";
        string filename = stream.str();

        if(cv::imwrite(filename, depth))
        {
            cout << filename<<" Saved."<<endl;
        }

        saveDepth = false;
    }
}


void keyboardEventOccured(const visualization::KeyboardEvent& event, void* nothing)
{
    if(event.getKeySym() == "space"&& event.keyDown())
    {
        saveCloud = true;
        saveImg = true;
        saveDepth = true;
        filesNum_++;
    }
}


boost::shared_ptr<visualization::CloudViewer> createViewer()
{
    boost::shared_ptr<visualization::CloudViewer> v(new visualization::CloudViewer("OpenNI viewer"));
    v->registerKeyboardCallback(keyboardEventOccured);
    return(v);
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "rgbd_collector");
    ros::NodeHandle nh;
    cout<< "Press space to record point cloud to a file."<<endl;

    cv::namedWindow("rgb", 0);
    cv::resizeWindow("rgb", 640,480);

    cv::namedWindow("depth", 0);
    cv::resizeWindow("depth", 640,480);

    viewer = createViewer();

    ros::Subscriber pointcloud_sub = nh.subscribe("/camera/depth/color/points", 1, cloudCB);
    ros::Subscriber img_sub_rgb = nh.subscribe("/camera/color/image_raw",1 , imageCallbackRGB);
    ros::Subscriber img_sub_depth = nh.subscribe("/camera/aligned_depth_to_color/image_raw",1 , imageCallbackDepth);

    ros::Rate rate(30.0);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}