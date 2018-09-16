//
// Created by leo on 18-9-8.
//

#include <include/Converter.h>
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

        mpGlobalMap = boost::make_shared<PointCloud>();
        mptCloudMapViewer = make_shared<thread>(bind(&PointCloudMapping::Run, this));
    }

    void PointCloudMapping::Shutdown()
    {
        {
            unique_lock<mutex> lock(mMutexShutdown);
            mbShutDownFlag = true;
            mConKeyFrameUpdated.notify_one();
        }
        mptCloudMapViewer->join();
    }

    void
    PointCloudMapping::InsertKeyFrame(KeyFrame *pKF, cv::Mat &color, cv::Mat &depth, int id, vector<KeyFrame *> vpKFs)
    {
        cout << "Pointcloud map receive a keyframe, id = " << id << endl;

        {
            unique_lock<mutex> lock(mMutexKeyFrameDeal);
            mvpKeyFrames.push_back(pKF);//mvpKeyFrames用来构建点云

            KeyPointCloud *pKPC = new KeyPointCloud;
            pKPC->mnID = id;
            pKPC->mT = ORB_SLAM2::Converter::toSE3Quat(pKF->GetPose());
            pKPC->mpPointCloud = GeneratePointCloud(pKF, color, depth);

            mvpKeyPointClouds.push_back(pKPC);//将关键帧转变成关键点云
            mConKeyFrameUpdated.notify_one();
        }
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
        mpStatisticalFilter->setInputCloud(pPointCloud);
        mpStatisticalFilter->filter(*pPointCloud);
        mpVoxelFilter->setInputCloud(pPointCloud);
        mpVoxelFilter->filter(*pPointCloud);
        return pPointCloud;
    }


//只有这个View线程是用另外一个线程来跑的，但是这个View里面会有点云的合成
    void PointCloudMapping::Run()
    {

        while (1)
        {
            cout << "Viewer thread is running" << endl;
            {
                unique_lock<mutex> lock(mMutexShutdown);//关闭线程则弹出while循环
                if (mbShutDownFlag)
                {
                    break;
                }
            }

            {
                unique_lock<mutex> lock(mMutexKeyFrameUpdate);
                mConKeyFrameUpdated.wait(lock);//没有插入关键帧则阻塞线程
            }

            size_t N = 0;
            {
                unique_lock<mutex> lock(mMutexKeyFrameDeal);
                N = mvpKeyFrames.size();//判断当前关键帧队列有多长
                cout<<"The size of keyframe is "<<N<<endl;
            }

            if (mbLoopBusy || mbStop)//如果此时正在进行循环或者停止则跳过此循环，不进行点云生成
            {
                cout<<"The loop is busy or stopped"<<endl;
                continue;
            }

            if (musLastKeyFrameSize == N)//最后一帧已经处理
            {
                cout<<"The frame is the last frame"<<endl;
                mbPointCloudDealBusy = false;
                continue;
            }

            mbPointCloudDealBusy = true;

            {
                unique_lock<mutex> lock(mMutexGlobalMapDeal);
                for (size_t i = musLastKeyFrameSize; i < N; i++)
                {
                    PointCloud::Ptr pPointCloud(new PointCloud);
                    pcl::transformPointCloud(*(mvpKeyPointClouds[i]->mpPointCloud), *pPointCloud,
                                             mvpKeyPointClouds[i]->mT.inverse().matrix());

                    *mpGlobalMap += *pPointCloud;
                }
            }

            musLastKeyFrameSize = N;
            mbPointCloudDealBusy = false;
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
        cout << "GlobalMap save finished" << endl;
    }

    void PointCloudMapping::UpdateCloud()
    {
        if (!mbPointCloudDealBusy)
        {
            mbLoopBusy = true;
            cout << "Start close looping cloud point" << endl;
            PointCloud::Ptr pPointCloudGlobal(new PointCloud);
            for (int i = 0; i < mvpCurrentKFs.size(); i++)
            {
                for (int j = 0; j < mvpKeyPointClouds.size(); j++)
                {
                    if (mvpKeyPointClouds[j]->mnID == mvpCurrentKFs[i]->mnFrameId)
                    {
                        Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(mvpCurrentKFs[i]->GetPose());
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
                unique_lock<mutex> lock(mMutexGlobalMapDeal);
                mpVoxelFilter->setInputCloud(pPointCloudGlobal);
                mpVoxelFilter->filter(*mpGlobalMap);
            }

            mbLoopBusy = false;
        }
    }


}




























