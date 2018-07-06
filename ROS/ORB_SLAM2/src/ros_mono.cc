/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

#include <std_msgs/String.h>

#include <pcl_ros/point_cloud.h>  
#include <pcl_conversions/pcl_conversions.h>  
#include <sensor_msgs/PointCloud2.h>  

#include <pcl/filters/radius_outlier_removal.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <geometry_msgs/PoseArray.h>

#include "System.h"
#include "Converter.h"
using namespace std;
class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM,bool publish_frame):mpSLAM(pSLAM){
        _publish_frame = publish_frame;
	    _publish_pose = false;
    }
    void SaveMapCallback(const std_msgs::String::ConstPtr& msg);
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
    bool _publish_frame, _publish_pose;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();
     
     if(argc!=2)
    {
      cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_settings" << endl;   
        ros::shutdown();
        return 1;
    }

    const string &strSettingPath = argv[1];
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    if(!fSettings.isOpened())
    {
        cerr << "Failed to open setting file at: " << strSettingPath << endl;
        exit(-1);
    }
    const string strORBvoc = fSettings["Orb_Vocabulary"];
    const string strCamSet = fSettings["Cam_Setting"];
    int ReuseMap = fSettings["is_ReuseMap"];
    int showGUI = fSettings["is_showgui"];
    float ocmapRe = fSettings["tree_resolution"];


    bool bReuseMap = false;
    if (1 == ReuseMap)
        bReuseMap = true;

    bool isshowgui = false;
    if (1 == showGUI)
        isshowgui = true;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(strORBvoc,strCamSet,ORB_SLAM2::System::MONOCULAR,isshowgui, bReuseMap);
    cerr<<"initialized SLAM"<<endl;
   

    ImageGrabber igb(&SLAM,true);
    ros::NodeHandle nodeHandler;
    //show processed image
    image_transport::ImageTransport it(nodeHandler);

    octomap_msgs::Octomap msg_octomap;

    geometry_msgs::PoseArray msg_pose;

    octomap_msgs::Octomap msg_VF_octomap;

    sensor_msgs::PointCloud2 output;  

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;// 创建滤波器   
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    //ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
     ros::Subscriber sub = nodeHandler.subscribe("/usb_cam/image_raw", 1, &ImageGrabber::GrabImage,&igb);

     ros::Subscriber sub1 = nodeHandler.subscribe("/ORB_SLAM2/save_map", 1000, &ImageGrabber::SaveMapCallback, &igb);
    
     image_transport::Publisher pub_img = it.advertise("/ORB_SLAM2/image", 1);

     ros::Publisher Octomap_pub = nodeHandler.advertise<octomap_msgs::Octomap>("octomap_3d",1);

     ros::Publisher poseArray_pub = nodeHandler.advertise<geometry_msgs::PoseArray>("pose_array",1);
     
     ros::Publisher VF_Octomap_pub = nodeHandler.advertise<octomap_msgs::Octomap>("VF_octomap_3d",1);

      ros::Publisher pcl_pub = nodeHandler.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
  
   ros::Rate loop_rate(10);
  while (nodeHandler.ok())
   {
     // show the drawframe
     cv::Mat showframe = SLAM.getFrameDrawer();
     if(!showframe.empty())
     {
      sensor_msgs::ImagePtr showmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", showframe).toImageMsg();
      pub_img.publish(showmsg);
     }

    // show the octomap
    octomap::OcTree tree( ocmapRe );
    octomap::OcTree* cur_tree;
    cur_tree=&tree;
    std::vector<ORB_SLAM2::MapPoint*> showmappoint = SLAM.getmspmappoint();
    for(auto mp:showmappoint)
    {  
      cv::Mat mpWorldPos = mp->GetWorldPos();
      tree.updateNode( octomap::point3d(mpWorldPos.at<float>(0), mpWorldPos.at<float>(2), -mpWorldPos.at<float>(1)), true );
    }  
    // 更新octomap
    tree.updateInnerOccupancy();
    octomap_msgs::fullMapToMsg(*cur_tree, msg_octomap);
    msg_octomap.header.frame_id = "/map";
    msg_octomap.header.stamp = ros::Time::now();
    Octomap_pub.publish(msg_octomap);  


    // shoe Trajectory
    msg_pose.poses.clear();
    msg_pose.header.frame_id="/map";
    msg_pose.header.stamp= ros::Time::now();

    vector<ORB_SLAM2::KeyFrame*> vpKFs = SLAM.getsortVkf();
    for(size_t i=0; i<vpKFs.size(); i++)
    {
      ORB_SLAM2::KeyFrame* pKF = vpKFs[i];
      if(pKF->isBad())
            continue;
      cv::Mat R = pKF->GetRotation().t();
      vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);
      cv::Mat t = pKF->GetCameraCenter();
         
      geometry_msgs::Pose posetemp;
      geometry_msgs::Quaternion msg_Que;

      posetemp.position.x = t.at<float>(0);
      posetemp.position.y = t.at<float>(2);
      posetemp.position.z = -t.at<float>(1);
     
       msg_Que.x= q[0];
       msg_Que.y= q[1];
       msg_Que.z= q[2];
       msg_Que.w= q[3];

      posetemp.orientation = msg_Que;
      msg_pose.poses.push_back(posetemp);
    }
   poseArray_pub.publish(msg_pose);
 
   //show current pointoctomap
     octomap::OcTree vftree( ocmapRe );
     octomap::OcTree* cur_vf_tree;
     cur_vf_tree=&vftree;
    const vector<ORB_SLAM2::MapPoint*> vpRefMPs = SLAM.getvpKFp();
    cloud->width = vpRefMPs.size();
    cloud->height = 1;  
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height); 
    set<ORB_SLAM2::MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());
    unsigned int i=0;
   for(set<ORB_SLAM2::MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
   {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        cloud->points[i].x = pos.at<float>(0);  
        cloud->points[i].y = pos.at<float>(2);  
        cloud->points[i].z = -pos.at<float>(1);  
        i++;
    } 
     if(i>0)
     { 
       outrem.setInputCloud(cloud);              //设置输入点云
       outrem.setRadiusSearch(0.1);              //设置在半径的范围内找邻近点
       outrem.setMinNeighborsInRadius(3);       //设置查询点的邻近点集数小于3的删除
       outrem.filter(*cloud_filtered);//执行条件滤波，存储结果到cloud_filtered
       pcl::toROSMsg(*cloud_filtered, output);
       output.header.frame_id = "odom";
       pcl_pub.publish(output);  

    if(cloud_filtered->points.size()>0)
    {
     for (auto p:cloud_filtered->points)
     {
         vftree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
     }
     // 更新octomap
     vftree.updateInnerOccupancy();
     octomap_msgs::fullMapToMsg(*cur_vf_tree, msg_VF_octomap);
     msg_VF_octomap.header.frame_id = "/map";
     msg_VF_octomap.header.stamp = ros::Time::now();
     VF_Octomap_pub.publish(msg_VF_octomap);
    }
     }  

    ros::spinOnce();
    loop_rate.sleep();
   }
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("/home/shell/KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

   mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}


void ImageGrabber::SaveMapCallback(const std_msgs::String::ConstPtr& msg)
 {
  mpSLAM->SaveMapSLOT(msg->data.c_str());
}