//
// Created by leo on 18-10-30.
//

#include "rgbd_orb_slam2_ros_test.h"

SLAMSystem::SLAMSystem()
{
    mpSLAM = new ORB_SLAM2::System(argv1,argv2,ORB_SLAM2::System::RGBD,false);
}

int SLAMSystem::Run()
{
    //这些都是相关的路径
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv4);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);//调用LoadImages载入图像

    //检查彩色图和深度图
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    //记录追踪时间
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    //主循环
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)//进入循环
    {
        // 从文件中导入深度图和彩色图
        imRGB = cv::imread(string(argv3)+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);//读入RGB数据
        imD = cv::imread(string(argv3)+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);//读入深度数据
        double tframe = vTimestamps[ni];
        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv3) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        //将数据传入SLAM系统
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        mpSLAM->TrackRGBD(imRGB,imD,tframe);//将输入传入SLAM系统
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();//记录一次追踪的时间

        //计算时间相关
        vTimesTrack[ni]=ttrack;//记录跟踪时间
        //等待下一帧 TODO：什么原理没搞明白
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];
        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    //关闭所有线程
    mpSLAM->Shutdown();

    //记录追踪时间
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }

    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    //存储轨迹和关键帧
    mpSLAM->SaveTrajectoryTUM("CameraTrajectory.txt");
    mpSLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}


//负责载入图像
void SLAMSystem::LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_orb_slam2");
    ros::start();

    ros::NodeHandle node;
    ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("global_map",1);

    //专门开一个线程跑SLAM，然后通过ros发布到rivz
    SLAMSystem* mpSLAMSystem = new SLAMSystem;
    thread* mptSLAMRunning = new thread(&SLAMSystem::Run,mpSLAMSystem);

    while(ros::ok())
    {
        cout<<"ROS running..."<<endl;

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZRGBA>);
        global_map = mpSLAMSystem->mpSLAM->mpPointCloudMapping->GetGlobalMap();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_copy(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*global_map, *global_map_copy);

        sensor_msgs::PointCloud2 global_map_output;
        pcl::toROSMsg(*global_map_copy, global_map_output);
        global_map_output.header.frame_id = "camera_depth_frame";
        global_map_output.header.stamp = ros::Time::now();
        cloud_pub.publish(global_map_output);

        cout<<global_map->size()<<endl;

        ros::Rate rate(2);
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}