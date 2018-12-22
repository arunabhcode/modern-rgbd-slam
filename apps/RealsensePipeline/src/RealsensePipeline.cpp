// Copyright 2018 Arunabh Sharma

#include <fstream>
#include <iostream>

#include "cxxopts.hpp"
#include "nlohmann/json.hpp"
#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

#include "Room/RGBDSlam/Feature.h"

int main(int argc, char* argv[])
{
  if (argc < 3)
  {
    spdlog::info("Expected command: <app> -c <config file>");
    return 0;
  }

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

  cv::Mat img1 =
      cv::imread("/home/user/realsense-rgbd-slam/Data/img1_Color.png");
  cv::Mat img2 =
      cv::imread("/home/user/realsense-rgbd-slam/Data/img2_Color.png");

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
  std::string show_debug = json_inst["show_debug"];

  std::vector<cv::Point2f> pts1;
  std::vector<cv::Point2f> pts2;

  room::Feature feature_inst(show_debug);
  feature_inst.FeatureDetectionAndMatchORB(img1,
                                           img2,
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
                                           pts1,
                                           pts2);

  return 0;
}
