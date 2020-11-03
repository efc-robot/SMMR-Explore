#include <combine_grids/merging_pipeline.h>
#include <multirobot_map_merge/mapPair2tf.h>
#include <ros/ros.h>

namespace map_register
{

class MapRegister
{
private:
  ros::NodeHandle node_;

  /* parameters */
  // double confidence_threshold_;

  ros::ServiceServer service_;
  combine_grids::MergingPipeline pipeline_;

public:
  MapRegister();
  bool handle_function(multirobot_map_merge::mapPair2tf::Request &req, multirobot_map_merge::mapPair2tf::Response &res);
  
};

MapRegister::MapRegister()
{
  ros::NodeHandle private_nh("~");
  // private_nh.param("estimation_confidence", confidence_threshold_, 0.5);

  service_ = node_.advertiseService("GetMapTransform", &MapRegister::handle_function, this);
}

bool MapRegister::handle_function(multirobot_map_merge::mapPair2tf::Request &req, multirobot_map_merge::mapPair2tf::Response &res)
{
  cv::Mat transform;
  double confidence;
  if(!pipeline_.estimateTransform(req.grid1, req.grid2, transform, combine_grids::FeatureType::AKAZE, confidence)){
    res.confidence = 0;
    return true;
  }

  ROS_ASSERT(transform.type() == CV_64F);
  ROS_ASSERT((req.grid1.info.resolution-req.grid2.info.resolution)/req.grid1.info.resolution < 0.01);
  transform.at<double>(0, 2) = transform.at<double>(0, 2) * req.grid1.info.resolution;
  transform.at<double>(1, 2) = transform.at<double>(1, 2) * req.grid1.info.resolution;
  double a = transform.at<double>(0, 0);
  double b = transform.at<double>(1, 0);
  double scale = std::sqrt(a*a+b*b);
  confidence = confidence * std::min(scale, 1/scale);
  // transform = transform/scale;
  transform.at<double>(0, 0) = transform.at<double>(0, 0)/scale;
  transform.at<double>(1, 0) = transform.at<double>(1, 0)/scale;
  transform.at<double>(0, 1) = transform.at<double>(0, 1)/scale;
  transform.at<double>(1, 1) = transform.at<double>(1, 1)/scale;
  std::cout << "transform: " << transform << std::endl;

  cv::Mat map1_to_origin1 = cv::Mat::eye(3,3,CV_64F);
  map1_to_origin1.at<double>(0, 2) = req.grid1.info.origin.position.x;
  map1_to_origin1.at<double>(1, 2) = req.grid1.info.origin.position.y;
  cv::Mat map2_to_origin2 = cv::Mat::eye(3,3,CV_64F);
  map2_to_origin2.at<double>(0, 2) = req.grid2.info.origin.position.x;
  map2_to_origin2.at<double>(1, 2) = req.grid2.info.origin.position.y;

  cv::Mat origin1_to_map1;
  cv::invert(map1_to_origin1, origin1_to_map1);
  transform = map2_to_origin2 * transform * origin1_to_map1;
  std::cout << "map2_to_origin2: " << map2_to_origin2 << std::endl;
  std::cout << "origin1_to_map1: " << origin1_to_map1 << std::endl;
  std::cout << "transform_new: " << transform << std::endl;

  res.transform.translation.x = transform.at<double>(0, 2);
  res.transform.translation.y = transform.at<double>(1, 2);
  res.transform.translation.z = 0.;

  // our rotation is in fact only 2D, thus quaternion can be simplified
  a = transform.at<double>(0, 0);
  b = transform.at<double>(1, 0);
  res.transform.rotation.w = std::sqrt(2. + 2. * a) * 0.5;
  res.transform.rotation.x = 0.;
  res.transform.rotation.y = 0.;
  res.transform.rotation.z = std::copysign(std::sqrt(2. - 2. * a) * 0.5, b);
  res.confidence = confidence;
  std::cout << "confidence: " << confidence << std::endl;

  return true;
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_register");
  // this package is still in development -- start wil debugging enabled
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  map_register::MapRegister map_registration;
  ros::spin();
  return 0;
}