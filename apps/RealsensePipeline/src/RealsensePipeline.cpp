// Copyright 2018 Arunabh Sharma

#include <fstream>
#include <iostream>

#include "cxxopts.hpp"
#include "nlohmann/json.hpp"
#include "opencv2/opencv.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

#include "Room/RGBDSlam/Feature.h"
#include "Room/RGBDSlam/Odometry.h"

int main(int argc, char* argv[])
{
  if (argc < 3)
  {
    spdlog::info("Expected command: <app> -c <config file>");
    return 0;
  }
  spdlog::set_level(spdlog::level::trace);
  cxxopts::Options options("realsense pipeline", "offline realsense pipeline");
  options.add_options()("c,config",
                        "Name of config file including path",
                        cxxopts::value<std::string>());
  cxxopts::ParseResult result = options.parse(argc, argv);

  std::string config_file_str = result["c"].as<std::string>();
  std::ifstream config_file(config_file_str, std::ios::in | std::ios::binary);
  if (!config_file.is_open())
  {
    spdlog::error("Unable to open config file");
    return 0;
  }
  nlohmann::json json_inst;
  config_file >> json_inst;

  cv::Mat color0 =
      cv::imread("/home/user/realsense-rgbd-slam/Data/1_color.png");
  cv::Mat color1 =
      cv::imread("/home/user/realsense-rgbd-slam/Data/2_color.png");

  cv::Mat depth0 = cv::imread("/home/user/realsense-rgbd-slam/Data/1_depth.png",
                              cv::IMREAD_ANYDEPTH);
  cv::Mat depth1 = cv::imread("/home/user/realsense-rgbd-slam/Data/2_depth.png",
                              cv::IMREAD_ANYDEPTH);

  depth0.convertTo(depth0, CV_32FC1);
  depth0 = depth0 / 5000;
  depth1.convertTo(depth1, CV_32FC1);
  depth1 = depth1 / 5000;
  // return 0;
  // Hardcode intrinsics for now

  float focal = 525.0f;
  cv::Point2d pp(319.5, 239.5);

  int nfeatures      = json_inst["nfeatures"];
  float scale_factor = json_inst["scale_factor"];
  int nlevels        = json_inst["nlevels"];
  int edge_threshold = json_inst["edge_threshold"];
  int first_level    = json_inst["first_level"];
  int WTA_K          = json_inst["WTA_K"];
  cv::ORB::ScoreType score_type =
      static_cast<cv::ORB::ScoreType>(json_inst["score_type"]);
  int patch_size         = json_inst["patch_size"];
  int fast_threshold     = json_inst["fast_threshold"];
  bool with_rotation     = json_inst["with_rotation"];
  bool with_scale        = json_inst["with_scale"];
  float threshold_factor = json_inst["threshold_factor"];
  float num_points       = json_inst["num_points"];
  std::string show_debug = json_inst["show_debug"];

  std::vector<cv::Point2f> pts0;
  std::vector<cv::Point2f> pts1;

  cv::Mat output_R, output_t;

  room::Feature feature_inst(show_debug);
  feature_inst.FeatureDetectionAndMatchORB(color0,
                                           color1,
                                           nfeatures,
                                           scale_factor,
                                           nlevels,
                                           edge_threshold,
                                           first_level,
                                           WTA_K,
                                           score_type,
                                           patch_size,
                                           fast_threshold,
                                           with_rotation,
                                           with_scale,
                                           threshold_factor,
                                           num_points,
                                           pts0,
                                           pts1);

  room::Odometry odometry_inst(show_debug);
  odometry_inst.RtEstimationICP(pts0,
                                pts1,
                                color0,
                                color1,
                                depth0,
                                depth1,
                                focal,
                                pp,
                                output_R,
                                output_t);

  spdlog::info("Output R = \n {}", output_R);
  spdlog::info("Output t = \n {}", output_t);

  // cv::Mat debug_R, debug_t;
  // odometry_inst.RtEstimationEssential(pts0, pts1, focal, pp, debug_R,
  // debug_t);

  // spdlog::info("Debug R = {}", debug_R);
  // spdlog::info("Debug t = {}", debug_t);

  return 0;
}
