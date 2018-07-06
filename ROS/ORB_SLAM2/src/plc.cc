#include <ros/ros.h>  
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include "System.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <tf/LinearMath/Quaternion.h>
int main (int argc, char **argv)  
{  
  ros::init (argc, argv, "pcl_create");  
  ros::NodeHandle nh;  
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);  
  ros::Publisher pcl_filter_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_filter_output", 1);  
  
  ros::Publisher Octomap_pub = nh.advertise<octomap_msgs::Octomap>("octomap_3d",1);
  ros::Publisher Octomap_filter_pub = nh.advertise<octomap_msgs::Octomap>("octomap_filter_3d",1);

  tf::Quaternion q;
  q.setRPY(0, 0,1.56);
  cerr<<"test q:"<<q.getAxis().getX()<<" "<<q.getAxis().getY()<<" "<<q.getAxis().getZ()<<q.getW()<<endl;

    

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2 output;  
  sensor_msgs::PointCloud2 filter_output;  

  //const string &pclfilename = "/home/shell/octmap.bt";
  const string &strSettingPath = "/home/shell/catkin_ws/src/ORB_SLAM2_Enhanced-master/map_setting/Setting.yaml";
  
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    if(!fSettings.isOpened())
    {
        cerr << "Failed to open setting file at: " << strSettingPath << endl;
        exit(-1);
    }
    const string strORBvoc = fSettings["Orb_Vocabulary"];
    const string strCamSet = fSettings["Cam_Setting"];

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(strORBvoc,strCamSet,ORB_SLAM2::System::MONOCULAR,false, true);
    cerr<<"initialized SLAM"<<endl;
    

    std::vector<ORB_SLAM2::MapPoint*> showmappoint = SLAM.getmspmappoint();
   cloud->width = showmappoint.size();
   cloud->height = 1;  
   cloud->is_dense = false;
   cloud->points.resize(cloud->width * cloud->height); 
 //声明octomap变量
    cout<<"copy data into octomap..."<<endl;
    // 创建八叉树对象，参数为分辨率，这里设成了0.05
    octomap::OcTree tree( 0.05 );
    octomap::OcTree filter_tree( 0.05 );
// origin_tree
  unsigned int i=0;
  for(auto mp:showmappoint)
  {  
     cv::Mat mpWorldPos = mp->GetWorldPos();
     cloud->points[i].x =  mpWorldPos.at<float>(0);  
     cloud->points[i].y =  mpWorldPos.at<float>(1);  
     cloud->points[i].z =  mpWorldPos.at<float>(2);  
   tree.updateNode( octomap::point3d(mpWorldPos.at<float>(0), mpWorldPos.at<float>(2), -mpWorldPos.at<float>(1)), true );
    i++;
  }  
  tree.updateInnerOccupancy();

//filter_tree
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;// 创建滤波器    
   outrem.setInputCloud(cloud);              //设置输入点云
   outrem.setRadiusSearch(0.1);              //设置在0.8半径的范围内找邻近点
   outrem.setMinNeighborsInRadius(3);       //设置查询点的邻近点集数小于2的删除
   outrem.filter (*cloud_filtered);//执行条件滤波，存储结果到cloud_filtered
  for (auto p:cloud_filtered->points)
     {
         // 将点云里的点插入到octomap中
         filter_tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
     }
      // 更新octomap
   filter_tree.updateInnerOccupancy();
    // 存储octomap
   // tree.writeBinary( pclfilename );
    cout<<"done."<<endl; 

    octomap::OcTree* cur_tree;
    octomap::OcTree* cur_filter_tree;
    octomap_msgs::Octomap msg_octomap;
    octomap_msgs::Octomap msg_filter_octomap;
    cur_tree=&tree;
    cur_filter_tree=&filter_tree;

     octomap_msgs::fullMapToMsg(*cur_tree, msg_octomap);
     msg_octomap.header.frame_id = "/map";
     msg_octomap.header.stamp = ros::Time::now();

     octomap_msgs::fullMapToMsg(*cur_filter_tree, msg_filter_octomap);
     msg_filter_octomap.header.frame_id = "/map";
     msg_filter_octomap.header.stamp = ros::Time::now();
     
  pcl::toROSMsg(*cloud, output);  
  pcl::toROSMsg(*cloud_filtered, filter_output); 
  output.header.frame_id = "turtle1";
  filter_output.header.frame_id = "odom";
  ros::Rate loop_rate(1);  
  while (ros::ok())  
  {  

    outrem.setInputCloud(cloud);              //设置输入点云
   outrem.setRadiusSearch(0.1);              //设置在0.8半径的范围内找邻近点
   outrem.setMinNeighborsInRadius(3);       //设置查询点的邻近点集数小于2的删除
   outrem.filter (*cloud_filtered);//执行条件滤波，存储结果到cloud_filtered
   pcl::toROSMsg(*cloud_filtered, filter_output); 
   filter_output.header.frame_id = "odom";
    Octomap_pub.publish(msg_octomap);
    Octomap_filter_pub.publish(msg_filter_octomap);
    pcl_pub.publish(output);  
    pcl_filter_pub.publish(filter_output);
    ros::spinOnce();  
    loop_rate.sleep();  
  }  
  return 0;  
}  