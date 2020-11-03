#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include<tf/transform_listener.h>
#include<laser_geometry/laser_geometry.h>
#include<iostream>
#include<sstream>
#include"std_msgs/String.h"
#include "icp_registration/mapdata.h"         // 该头文件是之前创建的 AddTwoInts.src 自动生成的
#include "icp_registration/Laserdata.h" 
using namespace std;

class ICPclient
{
public:
    ICPclient()
    {
        cout<<"construct"<<endl;
        rob1_listener = nh.subscribe("/robot1/scan_raw", 20, &ICPclient::scan2point_rob1, this);
        rob2_listener = nh.subscribe("/robot2/scan_raw", 20, &ICPclient::scan2point_rob2, this);
        pcl_pub = nh.serviceClient<icp_registration::mapdata>("doing_icp");
        laser_pub = nh.serviceClient<icp_registration::Laserdata>("laser_icp");

// cout<<"announce"<<endl;
//     pcl::PointCloud<pcl::PointXYZ> cloud;
//     sensor_msgs::PointCloud2 output;
//     cloud.width  = 100;
//     cloud.height = 1;
//     cloud.points.resize(cloud.width * cloud.height);
//     for (size_t i = 0; i < 100; ++i)
//     {
//         cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
//         cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
//         cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
//         //cout<<cloud.points[i].x<<cloud.points[i].y<<cloud.points[i].z<<endl;
//     }
//     pcl::toROSMsg(cloud, output);
//     output.header.frame_id = "odom";
//         pcl::PointCloud<pcl::PointXYZ> cloud2;
//         cloud2 = cloud;
//         // for (size_t i = 0; i < cloud2.points.size (); ++i)
//         // {
//         //     cloud2.points[i].x = cloud.points[i].x + 2.0f;
//         // }
//         // cout<<"------------start-----------"<<endl;
//         // for (size_t i = 0; i < cloud2.points.size (); ++i)
//         // {
//         //     cout<<cloud2.points[i].x - cloud.points[i].x <<endl;;
//         // }
//         //  cout<<"------------end-----------"<<endl;
//         sensor_msgs::PointCloud2 output2;
//         pcl::toROSMsg(cloud2, output2);
//         output.header.frame_id = "odom";
//         icp_registration::mapdata srv;
//         srv.request.mapdata=output;
//         srv.request.mapdata2=output2;
//         // cout<<"ready"<<output<<output2<<endl;
//         if(pcl_pub.call(srv))
//         {
//             ROS_INFO("send service success!");
//         }
//         else{
//             ROS_INFO("ERROR!!!!!");
//         }
    //cout<<"end"<<endl;
    }
    void scan2point_rob1(sensor_msgs::LaserScan laser){
        
        // sensor_msgs::PointCloud2 cloud;
        // laser_geometry::LaserProjection projecter;
        // tf::TransformListener tfListener;
        // cout<<"R   1"<<endl;
        // tfListener.waitForTransform(laser.header.frame_id,"robot1/base_footprint",laser.header.stamp+ros::Duration().fromSec(laser.ranges.size()*laser.time_increment),ros::Duration(1.0));
        // projecter.transformLaserScanToPointCloud(laser.header.frame_id,laser, cloud,tfListener);
        // cout<<"TS  1!"<<endl;
        // srv.request.mapdata=cloud;
        Laser_srv.request.laser1=laser;
        rob1_laser=true;

        cout<<"--------------rob1:"<<rob1_laser<<"    rob2:"<<rob2_laser<<endl;
        if((rob1_laser) && (rob2_laser)){
        if(laser_pub.call(Laser_srv)/*pcl_pub.call(srv)*/)
        {
            ROS_INFO("send service success!");
            rob1_laser=false;
            rob2_laser=false;
        }
        else
        {
            cout<<"wait for rob2"<<endl;
        }
        }

    }
     void scan2point_rob2(sensor_msgs::LaserScan laser){
        
        // sensor_msgs::PointCloud2 cloud;
        // laser_geometry::LaserProjection projecter;
        // tf::TransformListener tfListener;
        // cout<<"R   2"<<endl;
        // tfListener.waitForTransform(laser.header.frame_id,"robot2/base_footprint",laser.header.stamp+ros::Duration().fromSec(laser.ranges.size()*laser.time_increment),ros::Duration(1.0));
        // projecter.transformLaserScanToPointCloud(laser.header.frame_id,laser, cloud,tfListener);
        // cout<<"TS  2!"<<endl;
        // srv.request.mapdata2=cloud;
        Laser_srv.request.laser2=laser;
        rob2_laser=true;

        if((rob1_laser) && (rob2_laser)){
        if(laser_pub.call(Laser_srv)/*pcl_pub.call(srv)*/)
        {
            ROS_INFO("send service success!");
            rob1_laser=false;
            rob2_laser=false;
        }
        else
        {
            cout<<"wait for rob1"<<endl;
        }
        }

    }

protected:
    ros::NodeHandle nh;
    ros::ServiceClient pcl_pub;
    ros::ServiceClient laser_pub;
    ros::Subscriber rob1_listener;
    ros::Subscriber rob2_listener;
    icp_registration::mapdata srv;
    icp_registration::Laserdata Laser_srv;
    bool rob1_laser=false;
    bool rob2_laser=false;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "client_node");
    ICPclient client;
    ros::spin();

    return 0;
}
