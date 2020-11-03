#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>
  
nav_msgs::OccupancyGrid CartographerMap;
nav_msgs::OccupancyGrid grid_;
unsigned char* cost_translation_table_ = new unsigned char[256];

void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  CartographerMap=*msg;
  // ROS_INFO("gointo callback");
}


int main(int argc, char **argv){

	ros::init(argc, argv, "cartographer2discrete");
	ros::NodeHandle n;
  ros::Subscriber sub=n.subscribe("cartographerMap", 1, mapCallBack);	
  ros::Publisher  pub=n.advertise<nav_msgs::OccupancyGrid>("map", 1);
  ros::Rate loop_rate(1);

  int lower_threshold = 50;
  int upper_threshold = 51;

  std::string ns;
  

  ns=ros::this_node::getName();
  // ros::param::param<int>(ns+"/threshold", threshold, 5);

  
  if (cost_translation_table_ != NULL)
  {
    // special values:
    for (int i = 0; i < lower_threshold; i++)
    {
      cost_translation_table_[ i ] = 0; 
    }
    for (int i = lower_threshold; i < upper_threshold; i++)
    {
      cost_translation_table_[ i ] = 255;
    }
    for (int i = upper_threshold; i < 255; i++)
    {
      cost_translation_table_[ i ] = 100;
    }
    cost_translation_table_[255]   = 255;    // unknown
  }


	while(ros::ok())
	{
    ros::spinOnce();
    grid_.header = CartographerMap.header;
    grid_.info = CartographerMap.info;
    grid_.data.resize(CartographerMap.info.width * CartographerMap.info.height);


    for (unsigned int i = 0; i < grid_.data.size(); i++)
    {
      unsigned char temp = CartographerMap.data[ i ];
      
      grid_.data[i] = cost_translation_table_[int(temp)];
      // if(temp == 255){ROS_INFO("%d -> %d: using %d", int(temp), cost_translation_table_[int(temp)], cost_translation_table_[255]);}
    }

    pub.publish(grid_);
    loop_rate.sleep();
  }

  return 0;
}
