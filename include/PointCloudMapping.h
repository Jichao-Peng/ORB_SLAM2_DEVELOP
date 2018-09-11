//
// Created by leo on 18-9-8.
//

#ifndef ORB_SLAM2_POINTCLOUDMAPPING_H
#define ORB_SLAM2_POINTCLOUDMAPPING_H



#include <condition_variable>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

#include "System.h"

using namespace ORB_SLAM2;

namespace ORB_SLAM2
{
    class KeyPointCloud
    {
        typedef pcl::PointXYZRGBA PointT;
        typedef pcl::PointCloud<PointT> PointCloud;

    public:
        PointCloud::Ptr mpPointCloud;
        Eigen::Isometry3d mT;
        int mnID;
    };
}


class PointCloudMapping
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PointCloudMapping(double resolution ,double meank ,double thresh);
    void InsertKeyFrame(KeyFrame *pKF, cv::Mat &color, cv::Mat &depth, int id, vector<KeyFrame *> vpKFs);
    void Shutdown();
    void Viewer();
    void UpdateCloud();
    void Save();

    vector<KeyFrame*> mvpCurrentKFs;
    bool mbPointCloudDealBusy;
    bool mbLoopBusy;
    bool mbStop = false;


protected:
    PointCloud::Ptr GeneratePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);

    shared_ptr<thread>  mptCloudMapViewer;
    PointCloud::Ptr mpGlobalMap;
    bool mbShutDownFlag =false;
    mutex mMutexShutdown;
    mutex mMutexKeyFrameDeal;
    mutex mMutexKeyFrameUpdate;
    condition_variable  mConKeyFrameUpdated;

    vector<KeyPointCloud*> mvpKeyPointClouds;
    vector<KeyFrame*>  mvpKeyFrames;

    unsigned short musLastKeyFrameSize = 0;

    double mResolution = 0.1;
    double mMeank = 50;
    double mThresh = 1;

    pcl::StatisticalOutlierRemoval<PointT>* mpStatisticalFilter;
    pcl::VoxelGrid<PointT>* mpVoxelFilter;
};

#endif //ORB_SLAM2_POINTCLOUDMAPPING_H