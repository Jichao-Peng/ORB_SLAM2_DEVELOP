//
// Created by leo on 18-10-30.
//

#ifndef PROJECT_RGBD_ORB_SLAM2_ROS_TEST_H
#define PROJECT_RGBD_ORB_SLAM2_ROS_TEST_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "ORB_SLAM2/include/System.h"

using namespace std;

class SLAMSystem
{
public:
    SLAMSystem();

    ORB_SLAM2::System* mpSLAM;
    int Run();
    void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                    vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

private:
    string argv1 = "/home/leo/Desktop/orb_slam2_project/src/orb_slam2/ORB_SLAM2/Vocabulary/ORBvoc.txt";
    string argv2 = "/home/leo/Desktop/orb_slam2_project/src/orb_slam2/src/TUM1.yaml";
    string argv3 = "/home/leo/Desktop/Data/rgbd_dataset_freiburg3_long_office_household";
    string argv4 = "/home/leo/Desktop/Data/rgbd_dataset_freiburg3_long_office_household/associations.txt";
};


#endif //PROJECT_RGBD_ORB_SLAM2_ROS_TEST_H
