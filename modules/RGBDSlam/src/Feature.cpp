// Copyright 2018 Arunabh Sharma

#include "Room/RGBDSlam/Feature.h"

#include <string>
#include <vector>

#include "opencv2/opencv.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "spdlog/spdlog.h"

#include "Room/RGBDSlam/Frame.h"

namespace room
{
Feature::Feature(const std::string show_debug) : m_show_debug(show_debug)
{
}

void Feature::FeatureDetectionAndMatchORB(Frame& frame0,
                                          Frame& frame1,
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
                                          const int num_points)
{
    std::vector<cv::KeyPoint> keypoints_0;
    std::vector<cv::KeyPoint> keypoints_1;
    std::vector<cv::KeyPoint> kps_sub_0;
    std::vector<cv::KeyPoint> kps_sub_1;
    std::vector<cv::KeyPoint> keypoints_match_0;
    std::vector<cv::KeyPoint> keypoints_match_1;
    cv::Mat descriptors_0;
    cv::Mat descriptors_1;
    std::vector<cv::Point2f> pts0;
    std::vector<cv::Point2f> pts1;

    cv::Mat img0 = frame0.GetColor();
    cv::Mat img1 = frame1.GetColor();

    cv::Ptr<cv::Feature2D> orb_inst = cv::ORB::create(nfeatures,
                                                      scale_factor,
                                                      nlevels,
                                                      edge_threshold,
                                                      first_level,
                                                      WTA_K,
                                                      score_type,
                                                      patch_size,
                                                      fast_threshold);

    orb_inst->detect(img0, keypoints_0);
    orb_inst->detect(img1, keypoints_1);

    kps_sub_0 = ANMS(keypoints_0, num_points);
    kps_sub_1 = ANMS(keypoints_1, num_points);

    orb_inst->compute(img0, kps_sub_0, descriptors_0);
    orb_inst->compute(img1, kps_sub_1, descriptors_1);

    std::vector<cv::DMatch> matches01;
    std::vector<cv::DMatch> good_matches;

    cv::Ptr<cv::DescriptorMatcher> matcher =
        cv::DescriptorMatcher::create("BruteForce-Hamming");
    matcher->match(descriptors_0, descriptors_1, matches01);

    cv::xfeatures2d::matchGMS(img0.size(),
                              img1.size(),
                              kps_sub_0,
                              kps_sub_1,
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

    cv::KeyPoint::convert(kps_sub_0, pts0, kp0_idxs);
    cv::KeyPoint::convert(kps_sub_1, pts1, kp1_idxs);

    frame0.SetKeypoints(pts0);
    frame1.SetKeypoints(pts1);

    if (m_show_debug == "ALL" || m_show_debug == "ORB_MATCHES")
    {
        cv::Mat img_matches;
        cv::drawMatches(img0,
                        kps_sub_0,
                        img1,
                        kps_sub_1,
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
}  // namespace room
