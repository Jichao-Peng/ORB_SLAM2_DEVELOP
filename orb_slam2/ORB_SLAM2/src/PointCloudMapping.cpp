//
// Created by leo on 18-9-8.
//

#include <ORB_SLAM2/include/Converter.h>
#include "PointCloudMapping.h"

namespace ORB_SLAM2
{
    PointCloudMapping::PointCloudMapping(double resolution, double meank, double thresh) :
            mResolution(resolution),
            mMeank(meank),
            mThresh(thresh)
    {
        mpStatisticalFilter = new pcl::StatisticalOutlierRemoval<PointT>();
        mpStatisticalFilter->setMeanK(mMeank);
        mpStatisticalFilter->setStddevMulThresh(mThresh);

        mpVoxelFilter = new pcl::VoxelGrid<PointT>();
        mpVoxelFilter->setLeafSize(mResolution, mResolution, mResolution);

        mpUniformFilter = new pcl::UniformSampling<PointT>();
        mpUniformFilter->setRadiusSearch(mResolution);


        mpGlobalMap = boost::make_shared<PointCloud>();
        mpGlobalMap->header.frame_id = "camera_depth_frame";
    }

    void PointCloudMapping::RequestFinish()
    {
        {
            unique_lock<mutex> lock(mMutexShutdown);//【锁】
            mbShutDownFlag = true;
            mConKeyPointCloudUpdated.notify_one();
        }
    }

    //InsertKeyFrame是放在Tracking线程中的，因此我感觉不能够在这里阻碍Tracking线程的进行
    //TODO:这一部分可以开一个临时线程，不占用Tracking的线程资源
    void PointCloudMapping::InsertKeyCloudPoint(KeyFrame *pKF, cv::Mat &color, cv::Mat &depth, int id)
    {
        unique_lock<mutex> lock(mMutexKeyPointCloudDeal);//【锁】
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        KeyPointCloud *pKPC = new KeyPointCloud;
        pKPC->mnID = id;
        pKPC->mT = ORB_SLAM2::Converter::toSE3Quat(pKF->GetPose());
        pKPC->mpPointCloud = GeneratePointCloud(pKF, color, depth);//生成点云也是在Tracking线程中进行的

        mvpKeyPointClouds.push_back(pKPC);//将关键帧转变成关键点云

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double Time= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        cout << "[POINTCLOUDMAP] pointcloud map receive a keyframe, id = " << id <<", and the insert time is "<<Time<< endl;

        mKeyPointCloudSize++;
        mConKeyPointCloudUpdated.notify_one();
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloudMapping::GeneratePointCloud(KeyFrame *pKf, cv::Mat &color,
                                                                                  cv::Mat &depth)
    {
        PointCloud::Ptr pPointCloud(new PointCloud());;
        for (int m = 0; m < depth.rows; m += 3)
        {
            for (int n = 0; n < depth.cols; n += 3)
            {
                float d = depth.ptr<float>(m)[n];
                if (d == 0)
                    continue;
                PointT p;
                p.z = d;
                p.x = (n - pKf->cx) * p.z / pKf->fx;
                p.y = (m - pKf->cy) * p.z / pKf->fy;

                p.b = color.ptr<uchar>(m)[n * 3];
                p.g = color.ptr<uchar>(m)[n * 3 + 1];
                p.r = color.ptr<uchar>(m)[n * 3 + 2];

                pPointCloud->points.push_back(p);
            }
        }

        PointCloud::Ptr tmp1(new PointCloud());
        mpUniformFilter->setInputCloud(pPointCloud);
        pcl::PointCloud<int> KeyPointIndices;
        mpUniformFilter->compute(KeyPointIndices);
        pcl::copyPointCloud(*pPointCloud, KeyPointIndices.points, *tmp1);

        PointCloud::Ptr tmp2(new PointCloud());
        mpStatisticalFilter->setInputCloud(tmp1);//删除离群点
        mpStatisticalFilter->filter(*tmp2);
        pPointCloud->swap(*tmp2);

        return pPointCloud;
    }


    //只有这个View线程是用另外一个线程来跑的，但是这个View里面会有点云的合成
    void PointCloudMapping::Run()
    {
        while (1)
        {
            {
                unique_lock<mutex> lock(mMutexShutdown);//关闭线程则弹出while循环
                if (mbShutDownFlag)
                {
                    break;
                }
            }

            {
                unique_lock<mutex> lock(mMutexKeyPointCloudUpdated);
                mConKeyPointCloudUpdated.wait(lock);//没有插入关键帧则阻塞线程
                cout<<"flag1"<<endl;
            }

            if (mbLoopBusy)//如果此时正在进行循环或者停止则跳过此循环，不进行全局地图的添加,这就是为什么闭环的时候地图停止更新了
            {
                cout<<"The loop is busy or stopped"<<endl;
                continue;
            }

            if (mLastKeyPointCloudSize == mKeyPointCloudSize)//最后一帧已经处理
            {
                cout<<"The frame is the last frame"<<endl;
                continue;
            }

            for (size_t i = mLastKeyPointCloudSize; i < mKeyPointCloudSize; i++)
            {
                unique_lock<mutex> lock(mMutexKeyPointCloudDeal);//【锁】
                unique_lock<mutex> lock2(mMutexGlobalMapDeal);//【锁】
                PointCloud::Ptr pPointCloud(new PointCloud);
                PointCloud::Ptr pGlobalPointCloud(new PointCloud);
                cout<<"flag2"<<endl;
                pcl::transformPointCloud(*(mvpKeyPointClouds[i]->mpPointCloud), *pPointCloud,
                                     mvpKeyPointClouds[i]->mT.inverse().matrix());
                cout<<"flag3"<<endl;
                *pGlobalPointCloud = *mpGlobalMap + *pPointCloud;
                mpGlobalMap->swap(*pGlobalPointCloud);
                cout<<"flag4"<<endl;
            }
            if(mpGlobalMap->size()-mGlobalMapSize > 5000)
            {
                cout<<"flag5"<<endl;
                unique_lock<mutex> lock(mMutexKeyPointCloudDeal);//【锁】
                unique_lock<mutex> lock2(mMutexGlobalMapDeal);//【锁】

                mGlobalMapSize = mpGlobalMap->size();
                cout << "[POINTCLOUDMAP] the global map points' number (before filter) is " << mpGlobalMap->size();
                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                PointCloud::Ptr tmp1(new PointCloud());
                mpUniformFilter->setInputCloud(mpGlobalMap);
                pcl::PointCloud<int> KeyPointIndices;
                mpUniformFilter->compute(KeyPointIndices);
                pcl::copyPointCloud(*mpGlobalMap, KeyPointIndices.points, *tmp1);
                mpGlobalMap->swap(*tmp1);
                std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
//                    double SF_Time= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();//记录时间
                double SF_Time = 0;
                double VF_Time= std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t2).count();//记录时间
                cout << "    the global map points' number (after filter) is " << mpGlobalMap->size() <<"   The SF_Time is "<<SF_Time<<"   The VF_Time is "<<VF_Time<<endl;
            }
            cout<<"flag6"<<endl;
            mLastKeyPointCloudSize = mKeyPointCloudSize;
        }
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloudMapping::GetGlobalMap()
    {
        unique_lock<mutex> lock(mMutexGlobalMapDeal);
        return mpGlobalMap;
    }

    void PointCloudMapping::Save()
    {
        pcl::io::savePCDFile("result.pcd", *mpGlobalMap);
        cout << endl << "global map saved!" << endl;
    }

    //UpdateCloud是放在LoopClosing线程里跑的
    void PointCloudMapping::UpdateCloud(vector<KeyFrame*> vpKFs)
    {
        unique_lock<mutex> lock(mMutexKeyPointCloudDeal);//【锁】
        unique_lock<mutex> lock2(mMutexGlobalMapDeal);//【锁】
        mbLoopBusy = true;
        cout << "Start close looping cloud point" << endl;
        PointCloud::Ptr pPointCloudGlobal(new PointCloud);
        for (int i = 0; i < vpKFs.size(); i++)//这个更新规则其实很蠢，就是查询与关键点云对应的关键帧，然后根据关键帧的位置更新关键点云
        {
            for (int j = 0; j < mvpKeyPointClouds.size(); j++)
            {
                if (mvpKeyPointClouds[j]->mnID == vpKFs[i]->mnFrameId)
                {
                    cout<< "There is one key point cloud, the ID is "<<mvpKeyPointClouds[j]->mnID<<endl;
                    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(vpKFs[i]->GetPose());
                    cout<< "The origin T is "<<endl<<mvpKeyPointClouds[j]->mT.inverse().matrix()<<endl<<"The corrected T is "<<endl<<T.inverse().matrix()<<endl<<endl;
                    PointCloud::Ptr pPointCloud(new PointCloud);
                    pcl::transformPointCloud(*mvpKeyPointClouds[j]->mpPointCloud, *pPointCloud,
                                             T.inverse().matrix());
                    *pPointCloudGlobal += *pPointCloud;
                    continue;
                }
            }
        }
        cout << "Finish close looping cloud point" << endl;
        {
            PointCloud::Ptr tmp1(new PointCloud());
            mpUniformFilter->setInputCloud(pPointCloudGlobal);
            pcl::PointCloud<int> KeyPointIndices;
            mpUniformFilter->compute(KeyPointIndices);
            pcl::copyPointCloud(*pPointCloudGlobal, KeyPointIndices.points, *tmp1);

            PointCloud::Ptr tmp2(new PointCloud());
            mpStatisticalFilter->setInputCloud(tmp1);//删除离群点
            mpStatisticalFilter->filter(*tmp2);
            mpGlobalMap->swap(*tmp2);
        }
        mbLoopBusy = false;
    }
}




























