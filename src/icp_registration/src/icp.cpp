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
#include<Eigen/Dense>
#include "icp_registration/mapdata.h"         // 该头文件是之前创建的 AddTwoInts.src 自动生成的
#include "icp_registration/Laserdata.h" 
using namespace std;

class cloudHandler
{
public:
    cloudHandler()
    {
        // pcl_sub = nh.subscribe("pcl_create", 10, &cloudHandler::cloudCB, this);
        // pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_matched", 1);
        cout<<"construct"<<endl;
        pcl_sub = nh.advertiseService("doing_icp",  &cloudHandler::PointCloudICP,this);
        pcl_laser_sub=nh.advertiseService("laser_icp", &cloudHandler::LaserICP,this);
    }

    bool PointCloudICP(icp_registration::mapdata::Request &req,icp_registration::mapdata::Response &res){
        pcl::PointCloud<pcl::PointXYZ> cloud_1;
        pcl::fromROSMsg(req.mapdata,cloud_1);
        pcl::PointCloud<pcl::PointXYZ> cloud_2;
        pcl::fromROSMsg(req.mapdata2,cloud_2);

        std::cout << "guess:"<<req.guess<< std::endl;
        Eigen::Quaternionf qg = Eigen::Quaternionf(req.guess.rotation.w, req.guess.rotation.x, req.guess.rotation.y, req.guess.rotation.z);
        Eigen::Isometry3f guess(qg);
        guess(0,3) = req.guess.translation.x;
        guess(1,3) = req.guess.translation.y;
        guess(2,3) = req.guess.translation.z;

        Eigen::Matrix4f trans=ICPreg(cloud_1,cloud_2, guess.matrix());

        Eigen::Matrix3f R;
        R=trans.block<3,3>(0,0);
        Eigen::Quaternionf q = Eigen::Quaternionf(R);
        res.transformation.rotation.x=q.x();
        res.transformation.rotation.y=q.y();
        res.transformation.rotation.z=q.z();
        res.transformation.rotation.w=q.w();
        res.transformation.translation.x=trans(0,3);
        res.transformation.translation.y=trans(1,3);
        res.transformation.translation.z=trans(2,3);
        std::cout << "PointCloud ICP:"<<res.transformation<< std::endl;
        return true;
    }

    bool LaserICP(icp_registration::Laserdata::Request &req,icp_registration::Laserdata::Response &res){
        sensor_msgs::PointCloud2 cloud1;
        sensor_msgs::PointCloud2 cloud2;
        laser_geometry::LaserProjection projecter1;
        laser_geometry::LaserProjection projecter2;
        tf::TransformListener tfListener1;
        tf::TransformListener tfListener2;
        tfListener1.waitForTransform(req.laser1.header.frame_id,"robot1/base_footprint",req.laser1.header.stamp+ros::Duration().fromSec(req.laser1.ranges.size()*req.laser1.time_increment),ros::Duration(1.0));
        projecter1.transformLaserScanToPointCloud(req.laser1.header.frame_id,req.laser1, cloud1,tfListener1);
        tfListener2.waitForTransform(req.laser2.header.frame_id,"robot2/base_footprint",req.laser2.header.stamp+ros::Duration().fromSec(req.laser2.ranges.size()*req.laser2.time_increment),ros::Duration(1.0));
        projecter2.transformLaserScanToPointCloud(req.laser2.header.frame_id,req.laser2, cloud2,tfListener2);
        pcl::PointCloud<pcl::PointXYZ> cloud_1;
        pcl::fromROSMsg(cloud1,cloud_1);
        pcl::PointCloud<pcl::PointXYZ> cloud_2;
        pcl::fromROSMsg(cloud2,cloud_2);

        std::cout << "guess:"<<req.guess<< std::endl;
        Eigen::Quaternionf qg = Eigen::Quaternionf(req.guess.rotation.w, req.guess.rotation.x, req.guess.rotation.y, req.guess.rotation.z);
        Eigen::Isometry3f guess(qg);
        guess(0,3) = req.guess.translation.x;
        guess(1,3) = req.guess.translation.y;
        guess(2,3) = req.guess.translation.z;

        Eigen::Matrix4f trans=ICPreg(cloud_1,cloud_2, guess.matrix());

        Eigen::Matrix3f R;
        R=trans.block<3,3>(0,0);
        Eigen::Quaternionf q=Eigen::Quaternionf(R);
        res.transformation.rotation.x=q.x();
        res.transformation.rotation.y=q.y();
        res.transformation.rotation.z=q.z();
        res.transformation.rotation.w=q.w();
        res.transformation.translation.x=trans(0,3);
        res.transformation.translation.y=trans(1,3);
        res.transformation.translation.z=trans(2,3);
        std::cout << "Laser ICP:"<<res.transformation<< std::endl;
        return true;
    }

    Eigen::Matrix4f ICPreg( pcl::PointCloud<pcl::PointXYZ> cloud1, pcl::PointCloud<pcl::PointXYZ> cloud2, Eigen::Matrix4f guess){
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        pcl::PointCloud<pcl::PointXYZ> cloud_aligned;
        icp.setInputSource(cloud1.makeShared());
        icp.setInputTarget(cloud2.makeShared());
        icp.setMaxCorrespondenceDistance(5);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon (1e-12);
        icp.setEuclideanFitnessEpsilon(0.1);
        icp.align(cloud_aligned, guess);
        return icp.getFinalTransformation();
    }



    //     bool ICPreg(icp_registration::mapdata::Request &req,icp_registration::mapdata::Response &res){
    //     cout<<"respond!"<<endl;
    //     pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    //     pcl::PointCloud<pcl::PointXYZ> cloud_1;
    //     pcl::fromROSMsg(req.mapdata,cloud_1);
    //     pcl::PointCloud<pcl::PointXYZ> cloud_2;
    //     pcl::fromROSMsg(req.mapdata2,cloud_2);
    //     pcl::PointCloud<pcl::PointXYZ> cloud_aligned;
    //             cout<<"------------start-----------"<<endl;
    //     for (size_t i = 0; i < cloud_2.points.size (); ++i)
    //     {
    //         cout<<cloud_2.points[i].x - cloud_1.points[i].x <<endl;;
    //     }
    //      cout<<"------------end-----------"<<endl;
    //     icp.setInputSource(cloud_1.makeShared());
    //     icp.setInputTarget(cloud_2.makeShared());

    //     icp.setMaxCorrespondenceDistance(5);
    //     icp.setMaximumIterations(100);
    //     icp.setTransformationEpsilon (1e-12);
    //     icp.setEuclideanFitnessEpsilon(0.1);
    //     icp.align(cloud_aligned);
    //     Eigen::Matrix4f trans=icp.getFinalTransformation();
    //     // cout<<icp.getFinalTransformation()<<endl;
    //     cout<<trans<<endl;
    //     Eigen::Matrix3f R;
    //     Eigen::Vector3f T;
    //     R=trans.block<3,3>(0,0);
    //     T=trans.block<3,1>(0,3);
    //     // cout<<R<<endl<<T<<endl;
    //     // cout<<"llllllll"<<endl;
    //     Eigen::Quaternionf q=Eigen::Quaternionf(R);
    //     res.transformation.rotation.x=q.x();
    //     res.transformation.rotation.y=q.y();
    //     res.transformation.rotation.z=q.z();
    //     res.transformation.rotation.w=q.w();
    //     res.transformation.translation.x=trans(0,3);
    //     res.transformation.translation.y=trans(1,3);
    //     res.transformation.translation.z=trans(2,3);
    //     // for (int i=0;i<4;i++){
    //     //     for (int j=0;j<4;j++){
    //     //         cout<<trans(i,j)<<endl;
    //     //     }
    //     // }
    //     std::cout << res.transformation<< std::endl;
        
       
    //     return true;

    // }
//         pcl_pub = nh.serviceClient<icp_registration::mapdata>("doing_icp");
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
//         for (size_t i = 0; i < cloud2.points.size (); ++i)
//         {
//             cloud2.points[i].x = cloud.points[i].x + 0.7f;
//         }
// sensor_msgs::PointCloud2 output2;
// pcl::toROSMsg(cloud2, output2);
//         icp_registration::mapdata srv;
//         srv.request.mapdata=output;
//         srv.request.mapdata2=output2;
//         cout<<"ready"<<output<<output2<<endl;
//         if(pcl_pub.call(srv))
//         {
//             ROS_INFO("send service success!");
//         }
//         else{
//             ROS_INFO("ERROR!!!!!");
//         }
//     cout<<"end"<<endl;


    // void cloudCB(const sensor_msgs::PointCloud2 &input)
    // {
    //     pcl::PointCloud<pcl::PointXYZ> cloud_in;
    //     pcl::PointCloud<pcl::PointXYZ> cloud_out;
    //     pcl::PointCloud<pcl::PointXYZ> cloud_aligned;
    //     sensor_msgs::PointCloud2 output;

    //     pcl::fromROSMsg(input, cloud_in);

    //     cloud_out = cloud_in;

    //     for (size_t i = 0; i < cloud_in.points.size (); ++i)
    //     {
    //         cloud_out.points[i].x = cloud_in.points[i].x + 0.7f;
    //     }

    //     pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    //     icp.setInputSource(cloud_in.makeShared());
    //     icp.setInputTarget(cloud_out.makeShared());

    //     icp.setMaxCorrespondenceDistance(5);
    //     icp.setMaximumIterations(100);
    //     icp.setTransformationEpsilon (1e-12);
    //     icp.setEuclideanFitnessEpsilon(0.1);

    //     icp.align(cloud_aligned);
    //     std::cout << icp.getFinalTransformation() << std::endl;
    //     pcl::toROSMsg(cloud_aligned, output);
    //     pcl_pub.publish(output);
    // }

protected:
    ros::NodeHandle nh;
    // ros::Subscriber pcl_sub;
    // ros::Publisher pcl_pub;
    ros::ServiceServer pcl_sub;
    ros::ServiceServer pcl_laser_sub;
    // ros::ServiceClient pcl_pub;

};

main(int argc, char **argv)
{
    ros::init(argc, argv, "icp_node");
    cout<<"start"<<endl;
    cloudHandler handler;
    
    // ros::NodeHandle nh1;
    // ros::Publisher pcl_pub = nh1.advertise<sensor_msgs::PointCloud2> ("pcl_create", 100);
    // // ros::ServiceClient pcl_pub = nh1.advertise<sensor_msgs::PointCloud2> ("sending_pointcloud");
    // // ros::Publisher pcl_pub1 = nh1.advertise<std_msgs::String> ("test", 1000);
    // // std_msgs::String msg;
    // // std::stringstream ss;
    // // int count1=0;
    // // ss<<"hello world"<<count1;
    // // msg.data=ss.str();
    // pcl::PointCloud<pcl::PointXYZ> cloud;
    // sensor_msgs::PointCloud2 output;
 
    // // Fill in the cloud data
    // cloud.width  = 100;
    // cloud.height = 1;
    // cloud.points.resize(cloud.width * cloud.height);
 
    // for (size_t i = 0; i < 100; ++i)
    // {
    //     cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    //     cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    //     cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    //     //cout<<cloud.points[i].x<<cloud.points[i].y<<cloud.points[i].z<<endl;
    // }
 
    // //Convert the cloud to ROS message
    // pcl::toROSMsg(cloud, output);
    // output.header.frame_id = "odom";
 
    // ros::Rate loop_rate(1);
    // int count=0;
    // while (ros::ok())
    // {
    //     cout<<count<<endl;
    //     pcl_pub.publish(output);
    //     // pcl_pub1.publish(msg);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    //     count++;
    // }

    ros::spin();

    return 0;
}
