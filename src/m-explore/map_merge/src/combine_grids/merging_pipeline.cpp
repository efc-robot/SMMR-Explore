/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <combine_grids/grid_compositor.h>
#include <combine_grids/grid_warper.h>
#include <combine_grids/merging_pipeline.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include "estimation_internal.h"

namespace combine_grids
{
bool MergingPipeline::estimateTransforms(FeatureType feature_type,
                                         double confidence)
{
  std::vector<cv::detail::ImageFeatures> image_features;
  std::vector<cv::detail::MatchesInfo> pairwise_matches;
  std::vector<cv::detail::CameraParams> transforms;
  std::vector<int> good_indices;
  // TODO investigate value translation effect on features
  cv::Ptr<cv::detail::FeaturesFinder> finder =
      internal::chooseFeatureFinder(feature_type);
  cv::Ptr<cv::detail::FeaturesMatcher> matcher =
      cv::makePtr<cv::detail::AffineBestOf2NearestMatcher>();
  cv::Ptr<cv::detail::Estimator> estimator =
      cv::makePtr<cv::detail::AffineBasedEstimator>();
  cv::Ptr<cv::detail::BundleAdjusterBase> adjuster =
      cv::makePtr<cv::detail::BundleAdjusterAffinePartial>();

  if (images_.empty()) {
    return true;
  }

  /* find features in images */
  ROS_DEBUG("computing features");
  image_features.reserve(images_.size());
  for (const cv::Mat& image : images_) {
    image_features.emplace_back();
    if (!image.empty()) {
      (*finder)(image, image_features.back());
    }
  }
  finder->collectGarbage();

  /* find corespondent features */
  ROS_DEBUG("pairwise matching features");
  (*matcher)(image_features, pairwise_matches);
  matcher->collectGarbage();

#ifndef NDEBUG
  internal::writeDebugMatchingInfo(images_, image_features, pairwise_matches);
#endif

  /* use only matches that has enough confidence. leave out matches that are not
   * connected (small components) */
  good_indices = cv::detail::leaveBiggestComponent(
      image_features, pairwise_matches, static_cast<float>(confidence));

  // no match found. try set first non-empty grid as reference frame. we try to
  // avoid setting empty grid as reference frame, in case some maps never
  // arrive. If all is empty just set null transforms.
  if (good_indices.size() == 1) {
    transforms_.clear();
    transforms_.resize(images_.size());
    for (size_t i = 0; i < images_.size(); ++i) {
      if (!images_[i].empty()) {
        // set identity
        transforms_[i] = cv::Mat::eye(3, 3, CV_64F);
        break;
      }
    }
    return true;
  }

  /* estimate transform */
  ROS_DEBUG("calculating transforms in global reference frame");
  // note: currently used estimator never fails
  if (!(*estimator)(image_features, pairwise_matches, transforms)) {
    return false;
  }

  /* levmarq optimization */
  // openCV just accepts float transforms
  for (auto& transform : transforms) {
    transform.R.convertTo(transform.R, CV_32F);
  }
  ROS_DEBUG("optimizing global transforms");
  adjuster->setConfThresh(confidence);
  if (!(*adjuster)(image_features, pairwise_matches, transforms)) {
    ROS_WARN("Bundle adjusting failed. Could not estimate transforms.");
    return false;
  }

  transforms_.clear();
  transforms_.resize(images_.size());
  size_t i = 0;
  for (auto& j : good_indices) {
    // we want to work with transforms as doubles
    transforms[i].R.convertTo(transforms_[static_cast<size_t>(j)], CV_64F);
    ++i;
  }

  return true;
}

// checks whether given matrix is an identity, i.e. exactly appropriate Mat::eye
static inline bool isIdentity(const cv::Mat& matrix)
{
  if (matrix.empty()) {
    return false;
  }
  cv::MatExpr diff = matrix != cv::Mat::eye(matrix.size(), matrix.type());
  return cv::countNonZero(diff) == 0;
}

nav_msgs::OccupancyGrid::Ptr MergingPipeline::composeGrids(std::string world_frame_="/world")
{
  ROS_ASSERT(images_.size() == transforms_.size());
  ROS_ASSERT(images_.size() == grids_.size());

  if (images_.empty()) {
    return nullptr;
  }

  ROS_DEBUG("warping grids");
  internal::GridWarper warper;
  std::vector<cv::Mat> imgs_warped;
  imgs_warped.reserve(images_.size());
  std::vector<cv::Rect> rois;
  rois.reserve(images_.size());

  cv::Point2i merged_tl(0, 0);
  geometry_msgs::Pose origin;
  origin.orientation.w = 1.0;
  for (size_t i = 0; i < images_.size(); ++i) {
    if (!transforms_[i].empty() && !images_[i].empty()) {
      imgs_warped.emplace_back();
      rois.emplace_back(
          warper.warp(images_[i], transforms_[i], imgs_warped.back()));
      std::cout << "rois.back().tl()" << rois.back().tl() << std::endl;
      std::cout << "rois.back().br()" << rois.back().br() << std::endl;
      
      std::cout << "grids_[i]->header.frame_id" << grids_[i]->header.frame_id << std::endl;
      if(grids_[i]->header.frame_id == world_frame_ || "/" + grids_[i]->header.frame_id == world_frame_){
        origin = grids_[i]->info.origin;
        continue;
      }
      if(rois.back().tl().x < merged_tl.x){
        merged_tl.x = rois.back().tl().x;
      }
      if(rois.back().tl().y < merged_tl.y){
        merged_tl.y = rois.back().tl().y;
      }
    }
  }

  if (imgs_warped.empty()) {
    return nullptr;
  }

  ROS_DEBUG("compositing result grid");
  nav_msgs::OccupancyGrid::Ptr result;
  internal::GridCompositor compositor;
  result = compositor.compose(imgs_warped, rois);

  // set correct resolution to output grid. use resolution of identity (works
  // for estimated trasforms), or any resolution (works for know_init_positions)
  // - in that case all resolutions should be the same.
  float any_resolution = 0.0;
  for (size_t i = 0; i < transforms_.size(); ++i) {
    // check if this transform is the reference frame
    if (isIdentity(transforms_[i])) {
      result->info.resolution = grids_[i]->info.resolution;
      break;
    }
    if (grids_[i]) {
      any_resolution = grids_[i]->info.resolution;
    }
  }
  if (result->info.resolution <= 0.f) {
    result->info.resolution = any_resolution;
  }

  // set grid origin to its centre
  result->info.origin.position.x = origin.position.x + merged_tl.x * double(result->info.resolution);
  result->info.origin.position.y = origin.position.y + merged_tl.y * double(result->info.resolution);
  result->info.origin.orientation.w = 1.0;

  return result;
}

std::vector<geometry_msgs::Transform> MergingPipeline::getTransforms() const
{
  std::vector<geometry_msgs::Transform> result;
  result.reserve(transforms_.size());

  for (auto& transform : transforms_) {
    if (transform.empty()) {
      result.emplace_back();
      continue;
    }

    ROS_ASSERT(transform.type() == CV_64F);
    geometry_msgs::Transform ros_transform;
    ros_transform.translation.x = transform.at<double>(0, 2);
    ros_transform.translation.y = transform.at<double>(1, 2);
    ros_transform.translation.z = 0.;

    // our rotation is in fact only 2D, thus quaternion can be simplified
    double a = transform.at<double>(0, 0);
    double b = transform.at<double>(1, 0);
    ros_transform.rotation.w = std::sqrt(2. + 2. * a) * 0.5;
    ros_transform.rotation.x = 0.;
    ros_transform.rotation.y = 0.;
    ros_transform.rotation.z = std::copysign(std::sqrt(2. - 2. * a) * 0.5, b);

    result.push_back(ros_transform);
  }

  return result;
}

bool MergingPipeline::estimateTransform(nav_msgs::OccupancyGrid& grid1, nav_msgs::OccupancyGrid& grid2, cv::Mat& transform_12, FeatureType feature_type, double& confidence)
{
  srv_seq++;

  cv::Ptr<cv::detail::FeaturesFinder> finder = internal::chooseFeatureFinder(feature_type);
  cv::Ptr<cv::detail::FeaturesMatcher> matcher = cv::makePtr<cv::detail::AffineBestOf2NearestMatcher>();
  cv::Ptr<cv::detail::Estimator> estimator = cv::makePtr<cv::detail::AffineBasedEstimator>();
  cv::Ptr<cv::detail::BundleAdjusterBase> adjuster = cv::makePtr<cv::detail::BundleAdjusterAffinePartial>();
  
  cv::Mat image1(grid1.info.height, grid1.info.width, CV_8UC1, const_cast<signed char*>(grid1.data.data()));
  cv::Mat image2(grid2.info.height, grid2.info.width, CV_8UC1, const_cast<signed char*>(grid2.data.data()));
  // cv::imwrite("map1_" + std::to_string(srv_seq) + ".png", image1);
  // cv::imwrite("map2_" + std::to_string(srv_seq) + ".png", image2);

  /* find features in images */
  ROS_DEBUG("computing features");
  cv::detail::ImageFeatures image_features1, image_features2;
  (*finder)(image1, image_features1);
  (*finder)(image2, image_features2);
  finder->collectGarbage();
  std::cout << "image1 feature number: " << image_features1.keypoints.size() << "image2 feature number: " << image_features2.keypoints.size() << std::endl;
  if(image_features1.keypoints.size()<1 || image_features2.keypoints.size()<1)
    return false;


  /* find corespondent features */
  ROS_DEBUG("pairwise matching features");
  cv::detail::MatchesInfo pair_matches;
  (*matcher)(image_features1, image_features2, pair_matches);
  std::cout << " pair_matches.matches.size()= " << pair_matches.matches.size() << std::endl;
  std::cout << " pair_matches.num_inliers= " << pair_matches.num_inliers << std::endl;
  std::cout << " pair_matches.confidence= " << pair_matches.confidence << std::endl;
  pair_matches.H.convertTo(transform_12, CV_64F);
  confidence = pair_matches.confidence;
//   std::vector<cv::detail::ImageFeatures> image_features {image_features1, image_features2};
//   std::vector<cv::detail::MatchesInfo> pairwise_matches;
//   (*matcher)(image_features, pairwise_matches);
//   matcher->collectGarbage();
//   ROS_DEBUG("pairwise_matches size: %d", pairwise_matches.size());

// #ifndef NDEBUG
//   std::vector<cv::Mat> images {image1, image2};
//   internal::writeDebugMatchingInfo(images, image_features, pairwise_matches, srv_seq);
// #endif
 
//   /* use only matches that has enough confidence. leave out matches that are not
//    * connected (small components) */
//   std::vector<int> good_indices;
//   good_indices = cv::detail::leaveBiggestComponent(image_features, pairwise_matches, static_cast<float>(confidence));
//   ROS_DEBUG("good_indices size: %d", good_indices.size());

//   /* estimate transform */
//   ROS_DEBUG("calculating transforms in global reference frame");
//   std::vector<cv::detail::CameraParams> transforms;
//   // note: currently used estimator never fails
//   if (!(*estimator)(image_features, pairwise_matches, transforms)) {
//     return false;
//   }
//   ROS_DEBUG("transforms size: %d", transforms.size());
//   std::cout << transforms[1].R << std::endl;

//   /* levmarq optimization */
//   // openCV just accepts float transforms
//   for (auto& transform : transforms) {
//     transform.R.convertTo(transform.R, CV_32F);
//   }
//   ROS_DEBUG("optimizing global transforms");
//   adjuster->setConfThresh(confidence);
//   if (!(*adjuster)(image_features, pairwise_matches, transforms)) {
//     ROS_WARN("Bundle adjusting failed. Could not estimate transforms.");
//     return false;
//   }

//   if(transforms.size()!=2){
//     ROS_DEBUG("wrong transforms size: %d", transforms.size());
//     return false;
//   }
//   std::cout << transforms[1].R << std::endl;
//   transforms[1].R.convertTo(transform_12, CV_64F);
  std::cout << transform_12 << std::endl;
  if(transform_12.rows<3 || transform_12.cols<3)
    return false;
  return true;
}

}  // namespace combine_grids
