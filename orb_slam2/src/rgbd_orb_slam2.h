//
// Created by leo on 18-9-22.
//

#ifndef PROJECT_RGBD_ORB_SLAM2_H
#define PROJECT_RGBD_ORB_SLAM2_H

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/core/core.hpp>

#include "ORB_SLAM2/include/System.h"
#include "PointCloudMapping.h"

using namespace std;

class SLAMSystem
{
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
public:
    SLAMSystem(string image_topic, string depth_topic);

    void Run();
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

private:
    ros::NodeHandle node;
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub;
    message_filters::Synchronizer<sync_pol> synchronizer;
    ros::Publisher cloud_pub;
    ORB_SLAM2::System* mpSLAM;
};


#endif //PROJECT_RGBD_ORB_SLAM2_H
