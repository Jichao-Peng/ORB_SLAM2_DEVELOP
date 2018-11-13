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
#include <pcl/keypoints/uniform_sampling.h>

#include "System.h"

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

namespace ORB_SLAM2
{
    class PointCloudMapping
    {
    public:
        typedef pcl::PointXYZRGBA PointT;
        typedef pcl::PointCloud<PointT> PointCloud;

        PointCloudMapping(double resolution, double meank, double thresh);

        void InsertKeyCloudPoint(KeyFrame *pKF, cv::Mat &color, cv::Mat &depth, int id);

        void RequestFinish();

        void Run();

        void UpdateCloud(vector<KeyFrame*> vpKFs);

        void Save();

        PointCloud::Ptr GetGlobalMap();

        bool mbLoopBusy = false;


    protected:
        PointCloud::Ptr GeneratePointCloud(KeyFrame *kf, cv::Mat &color, cv::Mat &depth);

        PointCloud::Ptr mpGlobalMap;
        bool mbShutDownFlag = false;
        mutex mMutexShutdown;
        mutex mMutexKeyPointCloudDeal;
        mutex mMutexGlobalMapDeal;
        mutex mMutexKeyPointCloudUpdated;
        condition_variable mConKeyPointCloudUpdated;

        vector<KeyPointCloud *> mvpKeyPointClouds;

        double mResolution = 0.1;
        double mMeank = 50;
        double mThresh = 1;

        pcl::StatisticalOutlierRemoval<PointT> *mpStatisticalFilter;
        pcl::VoxelGrid<PointT> *mpVoxelFilter;
        pcl::UniformSampling<PointT> *mpUniformFilter;

        int mLastKeyPointCloudSize = 0;
        int mKeyPointCloudSize = 0;
        int mGlobalMapSize = 0;
    };

}
#endif //ORB_SLAM2_POINTCLOUDMAPPING_H
