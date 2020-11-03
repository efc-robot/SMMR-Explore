#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>

using namespace std;

nav_msgs::OccupancyGrid msg1;
nav_msgs::OccupancyGrid msg2;
nav_msgs::OccupancyGrid msg3;
nav_msgs::OccupancyGrid msg_merge;
nav_msgs::OccupancyGrid mapData1,mapData2,mapData3;

void mapCallBack1(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
mapData1=*msg;
}
void mapCallBack2(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
mapData2=*msg;
}
void mapCallBack3(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
mapData3=*msg;
}


signed char pixel_merge(signed char merged_pixel,signed char local_pixel)
{
  if(merged_pixel==-1){merged_pixel=local_pixel;}
  else if (merged_pixel==0)
  {
    if(local_pixel==100){merged_pixel=local_pixel;}
  }
  else if(merged_pixel==100){}
  return merged_pixel;
}

//nav_msgs::OccupancyGrid
void merge(nav_msgs::OccupancyGrid &result,std::vector<nav_msgs::OccupancyGrid> temp1,std::vector<float> transform)
{
 int size=temp1.size();
 float resolution=temp1.front().info.resolution;
 std::vector<nav_msgs::OccupancyGrid>::iterator it;
 int count=0;
 for(it=temp1.begin(); it!=temp1.end(); it++)
 {
  nav_msgs::OccupancyGrid it_map=*it;
  int x_leftdown=(transform[2*count]+it_map.info.origin.position.x)/resolution;//像素坐标
  int y_leftdown=(transform[2*count+1]+it_map.info.origin.position.y-6)/resolution;//像素坐标
  int row_start=(result.info.height/2+y_leftdown)*result.info.width+(result.info.width/2+x_leftdown);//每行起始坐标
  int row_end=row_start+it_map.info.width-1;//每行停止坐标
  for(int j=0;j<it_map.info.height;j++)//对小地图的每一列
  {
   for (int i=row_start;i<=row_end;i++)//对小地图的每一行对应的大地图中的位置
   {
    int k=j*it_map.info.width+i-row_start;
    result.data[i]=pixel_merge(result.data[i],it_map.data[k]);
   }
   row_start=row_start+result.info.width;
   row_end=row_start+it_map.info.width-1;
  }
  count++;
 }
 result.header.stamp = ros::Time::now(); 
 //return result;
}



int main(int argc, char **argv){
  //rviz 显示map1,map2,map_merge
	ros::init(argc,argv,"map_merge");
	ros::NodeHandle n;
  ros::Publisher pub=n.advertise<nav_msgs::OccupancyGrid>("/map_merge/map",1000);
  ros::Subscriber sub1= n.subscribe("/robot_1/map", 100 ,mapCallBack1);	
  ros::Subscriber sub2= n.subscribe("/robot_2/map", 100 ,mapCallBack2);
  ros::Subscriber sub3= n.subscribe("/robot_3/map", 100 ,mapCallBack3);
	ros::Rate loop_rate(100);

  // prefetch the param
  std::string ns;
  ns=ros::this_node::getName();

  float robot1_x, robot2_x, robot3_x, robot1_y, robot2_y, robot3_y;
  ros::param::param<float>(ns + "/robot1_x", robot1_x, 0.0);
  ros::param::param<float>(ns + "/robot2_x", robot2_x, 0.0); 
  ros::param::param<float>(ns + "/robot3_x", robot3_x, 0.0);
  ros::param::param<float>(ns + "/robot1_y", robot1_y, 0.0); 
  ros::param::param<float>(ns + "/robot2_y", robot2_y, -0.8); 
  ros::param::param<float>(ns + "/robot3_y", robot3_y, 0.8); 

  ROS_INFO("robot1_x: %3.1f", robot1_x);
  ROS_INFO("robot2_x: %3.1f", robot2_x);
  ROS_INFO("robot3_x: %3.1f", robot3_x);

	// ros::Subscriber sub=n.subscribe("show_topic",1000,show_callback);

  //我把消息全部变成全局变量了，方便在函数声明中调用
	//int count=0;
  nav_msgs::OccupancyGrid merged_map;
  nav_msgs::OccupancyGrid result;
  result.header.frame_id="robot_1/map";
  result.header.stamp = ros::Time::now(); 

  // result.info.resolution = mapData1.info.resolution;     
  result.info.resolution = 0.1;         // float32   // rosparam
  result.info.width      = 1000;        //  uint32   // rosparam
  result.info.height     = 1000;                     // rosparam
  int p[result.info.width*result.info.height];
  for(int i=0;i<result.info.width*result.info.height;i++)
  {
    p[i]=-1;
  }
  std::vector<signed char> a(p, p+result.info.width*result.info.height);
  result.data = a;
  result.info.origin.position.x=(-1.0)*result.info.width*result.info.resolution/2.0;
  result.info.origin.position.y=(-1.0)*result.info.height*result.info.resolution/2.0;
  std::vector<float> transform;
 
  transform.push_back(robot1_x);   // rosparam
  transform.push_back(robot1_y);   // rosparam
  
  transform.push_back(robot2_x);  // rosparam
  transform.push_back(robot2_y);  // rosparam

  transform.push_back(robot3_x);   // rosparam
  transform.push_back(robot3_y);   // rosparam：缺少旋转。

	while(ros::ok())
	{
    
    result.data = a;
    ros::spinOnce();
    std::vector<nav_msgs::OccupancyGrid> temp1;
    temp1.push_back(mapData1);
    temp1.push_back(mapData2); 
    temp1.push_back(mapData3);     
      //merged_map=
    ros::spinOnce();
    merge(result,temp1,transform);
    pub.publish(result);
    loop_rate.sleep();
	}


  // ros::init(argc, argv, "map_merge");
  // // this package is still in development -- start wil debugging enabled
  // 	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
  //                                    ros::console::levels::Debug)) {
  //   ros::console::notifyLoggerLevelsChanged();
  // 	}
  // 	map_merge::MapMerge map_merging;
	//设置参数
	//map_merging.subscriptions_size_=2;

  
	  
	//map_merging.spin();
	return 0;
}
