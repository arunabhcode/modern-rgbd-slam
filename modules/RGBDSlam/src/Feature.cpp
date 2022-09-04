// Copyright 2018 Arunabh Sharma

#include "RGBDSlam/Feature.h"

#include <string>
#include <vector>

#include "Components/Frame.h"
#include "opencv2/opencv.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "spdlog/spdlog.h"

namespace slam
{
Feature::Feature(const std::string show_debug,
                 const int nfeatures,
                 const float scale_factor,
                 const int nlevels,
                 const int edge_threshold,
                 const int first_level,
                 const int WTA_K,
                 const cv::ORB::ScoreType score_type,
                 const int patch_size,
                 const int fast_threshold,
                 const int num_points)
    : show_debug_(show_debug), num_points_(num_points)
{
  feature_md_inst_ = cv::ORB::create(nfeatures,
                                     scale_factor,
                                     nlevels,
                                     edge_threshold,
                                     first_level,
                                     WTA_K,
                                     score_type,
                                     patch_size,
                                     fast_threshold);
}

void Feature::FindFeatures(Frame& frame)
{
  std::vector<cv::KeyPoint> keypoints;
  feature_md_inst_->detect(frame.color_img_, keypoints);
  frame.features_       = ANMS(keypoints, num_points_);
  frame.features_found_ = true;
}

void Feature::FeatureMatch(Frame& frame0,
                           Frame& frame1,
                           const bool with_rotation,
                           const bool with_scale,
                           const float threshold_factor)
{
  cv::Mat descriptors_0;
  cv::Mat descriptors_1;

  feature_md_inst_->compute(frame0.color_img_, frame0.features_, descriptors_0);
  feature_md_inst_->compute(frame1.color_img_, frame1.features_, descriptors_1);

  std::vector<cv::DMatch> matches01;
  std::vector<cv::DMatch> good_matches;

  cv::Ptr<cv::DescriptorMatcher> matcher =
      cv::DescriptorMatcher::create("BruteForce-Hamming");
  matcher->match(descriptors_0, descriptors_1, matches01);

  cv::xfeatures2d::matchGMS(frame0.color_img_.size(),
                            frame1.color_img_.size(),
                            frame0.features_,
                            frame1.features_,
                            matches01,
                            good_matches,
                            with_rotation,
                            with_scale,
                            threshold_factor);

  std::vector<int> kp0_idxs;
  std::vector<int> kp1_idxs;
  for (std::size_t i = 0; i < good_matches.size(); i++)
  {
    kp0_idxs.push_back(good_matches[i].queryIdx);
    kp1_idxs.push_back(good_matches[i].trainIdx);
  }

  cv::KeyPoint::convert(frame0.features_, frame0.keypoints_, kp0_idxs);
  cv::KeyPoint::convert(frame1.features_, frame1.keypoints_, kp1_idxs);

  if (show_debug_ == "ALL" || show_debug_ == "ORB_MATCHES")
  {
    cv::Mat img_matches;
    cv::drawMatches(frame0.color_img_,
                    frame0.features_,
                    frame1.color_img_,
                    frame1.features_,
                    good_matches,
                    img_matches,
                    cv::Scalar::all(-1),
                    cv::Scalar::all(-1),
                    std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    spdlog::info("Displaying the matches on screen");
    cv::imshow("MatchesORB", img_matches);
    cv::waitKey(0);
    cv::destroyAllWindows();
  }
}

void Feature::FeatureTrack(Frame& frame0,
                           Frame& frame1,
                           const cv::Size win_size,
                           const int max_level,
                           std::vector<int>& kps_idx_track)
{
  if (!frame0.features_found_)
  {
    spdlog::warn("The previous frame doesn't have keypoints.");
    FindFeatures(frame0);
  }

  std::vector<unsigned char> kps_status_track_1;
  std::vector<float> kps_error_track_1;
  std::vector<cv::DMatch> good_matches;
  cv::KeyPoint::convert(frame0.features_, frame0.keypoints_);

  cv::calcOpticalFlowPyrLK(frame0.color_img_,
                           frame1.color_img_,
                           frame0.keypoints_,
                           frame1.keypoints_,
                           kps_status_track_1,
                           kps_error_track_1,
                           win_size,
                           max_level);
  cv::KeyPoint::convert(frame1.keypoints_, frame1.features_);
  for (std::size_t i = 0; i < frame1.keypoints_.size(); i++)
  {
    if (kps_status_track_1[i] == 1)
    {
      kps_idx_track.push_back(i);
      SPDLOG_INFO("Match idx = {}", i);
      good_matches.emplace_back(cv::DMatch(i, i, 1.0f));
    }
  }

  if (show_debug_ == "ALL" || show_debug_ == "OPTICAL_FLOW")
  {
    cv::Mat img_matches;
    cv::drawMatches(frame0.color_img_,
                    frame0.features_,
                    frame1.color_img_,
                    frame1.features_,
                    good_matches,
                    img_matches,
                    cv::Scalar::all(-1),
                    cv::Scalar::all(-1),
                    std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    spdlog::info("Displaying the matches on screen");
    cv::imshow("MatchesORB", img_matches);
    cv::waitKey(0);
    cv::destroyAllWindows();
  }
}

std::vector<cv::KeyPoint> Feature::ANMS(std::vector<cv::KeyPoint> keypoints,
                                        int num_points)
{
  std::vector<std::pair<float, int>> results;
  results.push_back(std::make_pair(FLT_MAX, 0));
  for (std::size_t i = 1; i < keypoints.size(); ++i)
  {  // for every keypoint we get the min distance to the previously visited
     // keypoints
    float min_dist = FLT_MAX;
    for (std::size_t j = 0; j < i; ++j)
    {
      float exp1     = (keypoints[j].pt.x - keypoints[i].pt.x);
      float exp2     = (keypoints[j].pt.y - keypoints[i].pt.y);
      float cur_dist = std::sqrt(exp1 * exp1 + exp2 * exp2);
      min_dist       = std::min(cur_dist, min_dist);
    }
    results.push_back(std::make_pair(min_dist, i));
  }
  std::sort(results.begin(),
            results.end(),
            [](const std::pair<float, int>& left,
               const std::pair<float, int>& right) {
              return left.first > right.first;
            });  // sorting by radius

  std::vector<cv::KeyPoint> kp;
  for (int i = 0; i < num_points; ++i)
    kp.push_back(
        keypoints[results[i].second]);  // extracting num_points keypoints

  return kp;
}
}  // namespace slam
