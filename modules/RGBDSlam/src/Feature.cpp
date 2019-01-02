// Copyright 2018 Arunabh Sharma

#include "Room/RGBDSlam/Feature.h"

#include <string>
#include <vector>

#include "opencv2/opencv.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "spdlog/spdlog.h"

namespace room
{
Feature::Feature(const std::string show_debug) : m_show_debug(show_debug)
{
}

void Feature::FeatureDetectionAndMatchORB(const cv::Mat& img1,
                                          const cv::Mat& img2,
                                          const int nfeatures,
                                          const float scale_factor,
                                          const int nlevels,
                                          const int edge_threshold,
                                          const int first_level,
                                          const int WTA_K,
                                          const cv::ORB::ScoreType score_type,
                                          const int patch_size,
                                          const int fast_threshold,
                                          const bool with_rotation,
                                          const bool with_scale,
                                          const float threshold_factor,
                                          std::vector<cv::Point2f>& pts1,
                                          std::vector<cv::Point2f>& pts2)
{
  std::vector<cv::KeyPoint> keypoints_1;
  std::vector<cv::KeyPoint> keypoints_2;
  std::vector<cv::KeyPoint> kps_sub_1;
  std::vector<cv::KeyPoint> kps_sub_2;
  std::vector<cv::KeyPoint> keypoints_match_1;
  std::vector<cv::KeyPoint> keypoints_match_2;
  cv::Mat descriptors_1;
  cv::Mat descriptors_2;
  cv::Ptr<cv::Feature2D> orb_inst = cv::ORB::create(nfeatures,
                                                    scale_factor,
                                                    nlevels,
                                                    edge_threshold,
                                                    first_level,
                                                    WTA_K,
                                                    score_type,
                                                    patch_size,
                                                    fast_threshold);

  orb_inst->detect(img1, keypoints_1);
  orb_inst->detect(img2, keypoints_2);

  kps_sub_1 = BrownANMS(keypoints_1, 3000);
  kps_sub_2 = BrownANMS(keypoints_2, 3000);

  // kps_sub_1 = keypoints_1;
  // kps_sub_2 = keypoints_2;

  orb_inst->compute(img1, kps_sub_1, descriptors_1);
  orb_inst->compute(img2, kps_sub_2, descriptors_2);

  std::vector<cv::DMatch> matches12;
  std::vector<cv::DMatch> good_matches;

  cv::Ptr<cv::DescriptorMatcher> matcher =
      cv::DescriptorMatcher::create("BruteForce-Hamming");
  matcher->match(descriptors_1, descriptors_2, matches12);

  cv::xfeatures2d::matchGMS(img1.size(),
                            img2.size(),
                            kps_sub_1,
                            kps_sub_2,
                            matches12,
                            good_matches,
                            with_rotation,
                            with_scale,
                            threshold_factor);

  std::vector<int> kp1_idxs;
  std::vector<int> kp2_idxs;
  for (std::size_t i = 0; i < good_matches.size(); i++)
  {
    kp1_idxs.push_back(good_matches[i].queryIdx);
    kp2_idxs.push_back(good_matches[i].trainIdx);
  }

  cv::KeyPoint::convert(kps_sub_1, pts1, kp1_idxs);
  cv::KeyPoint::convert(kps_sub_2, pts2, kp2_idxs);

  // spdlog::info(
  //     "pts size = {}, keypoint idxs = {}, matches size = {}, keypoint 1 size
  //     = "
  //     "{}",
  //     pts1.size(),
  //     kp1_idxs.size(),
  //     good_matches.size(),
  //     kps_sub_1.size());

  cv::Mat img_matches;
  cv::drawMatches(img1,
                  kps_sub_1,
                  img2,
                  kps_sub_2,
                  good_matches,
                  img_matches,
                  cv::Scalar::all(-1),
                  cv::Scalar::all(-1),
                  std::vector<char>(),
                  cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

  if (m_show_debug == "ALL" || m_show_debug == "ORB_MATCHES")
  {
    spdlog::info("Displaying the matches on screen");
    cv::imshow("MatchesORB", img_matches);
    cv::waitKey(0);
    cv::destroyAllWindows();
  }
}

std::vector<cv::KeyPoint> Feature::BrownANMS(
    std::vector<cv::KeyPoint> keypoints, int num_points)
{
  std::vector<std::pair<float, int>> results;
  results.push_back(std::make_pair(FLT_MAX, 0));
  for (std::size_t i = 1; i < keypoints.size(); ++i)
  {  // for every keypoint we get the min distance to the previously visited
     // keypoints
    float minDist = FLT_MAX;
    for (std::size_t j = 0; j < i; ++j)
    {
      float exp1    = (keypoints[j].pt.x - keypoints[i].pt.x);
      float exp2    = (keypoints[j].pt.y - keypoints[i].pt.y);
      float curDist = std::sqrt(exp1 * exp1 + exp2 * exp2);
      minDist       = std::min(curDist, minDist);
    }
    results.push_back(std::make_pair(minDist, i));
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
}  // namespace room
