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

  orb_inst->detectAndCompute(img1, cv::noArray(), keypoints_1, descriptors_1);
  orb_inst->detectAndCompute(img2, cv::noArray(), keypoints_2, descriptors_2);

  std::vector<cv::DMatch> matches12;
  std::vector<cv::DMatch> good_matches;

  cv::Ptr<cv::DescriptorMatcher> matcher =
      cv::DescriptorMatcher::create("BruteForce-Hamming");
  matcher->match(descriptors_1, descriptors_2, matches12);

  cv::xfeatures2d::matchGMS(img1.size(),
                            img2.size(),
                            keypoints_1,
                            keypoints_2,
                            matches12,
                            good_matches,
                            with_rotation,
                            with_scale,
                            threshold_factor);

  std::vector<int> kp1_idxs;
  std::vector<int> kp2_idxs;
  for (std::size_t i = 0; i < good_matches.size(); i++)
  {
    kp1_idxs.push_back(good_matches[i].trainIdx);
    kp2_idxs.push_back(good_matches[i].queryIdx);
  }

  cv::KeyPoint::convert(keypoints_1, pts1, kp1_idxs);
  cv::KeyPoint::convert(keypoints_2, pts2, kp2_idxs);

  // spdlog::info(
  //     "pts size = {}, keypoint idxs = {}, matches size = {}, keypoint 1 size
  //     = "
  //     "{}",
  //     pts1.size(),
  //     kp1_idxs.size(),
  //     good_matches.size(),
  //     keypoints_1.size());

  cv::Mat img_matches;
  cv::drawMatches(img1,
                  keypoints_1,
                  img2,
                  keypoints_2,
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
}  // namespace room
