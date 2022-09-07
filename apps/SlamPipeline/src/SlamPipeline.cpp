// Copyright 2018 Arunabh Sharma

#include <fstream>
#include <iostream>

#include "Components/Camera.h"
#include "Components/Frame.h"
#include "Components/Map.h"
#include "Components/Params.h"
#include "Components/Pose.h"
#include "RGBDSlam/Feature.h"
#include "RGBDSlam/Track.h"
#include "cxxopts.hpp"
#include "nlohmann/json.hpp"
#include "opencv2/opencv.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

int main(int argc, char* argv[])
{
  // if (argc < 3)
  // {
  //   spdlog::info("Expected command: <app> -c <config file>");
  //   return 0;
  // }
  spdlog::set_level(spdlog::level::info);
  spdlog::set_pattern("[%@][%l] %v");
  // cxxopts::Options options("modern pipeline", "offline modern pipeline");
  // options.add_options()("c,config", "Name of config file including path",
  // cxxopts::value<std::string>()); cxxopts::ParseResult result =
  // options.parse(argc, argv);

  // std::string config_file_str = result["c"].as<std::string>();
  std::string config_file_str =
      "/home/asharma/modern-rgbd-slam/apps/SlamPipeline/Offline.json";
  std::ifstream config_file(config_file_str, std::ios::in | std::ios::binary);
  if (!config_file.is_open())
  {
    spdlog::error("Unable to open config file");
    return 0;
  }
  else
  {
    spdlog::info("Opened config file");
  }
  nlohmann::json json_inst;
  slam::SlamParams params_inst;
  config_file >> json_inst;

  params_inst.nfeatures_      = json_inst["nfeatures"];
  params_inst.scale_factor_   = json_inst["scale_factor"];
  params_inst.nlevels_        = json_inst["nlevels"];
  params_inst.edge_threshold_ = json_inst["edge_threshold"];
  params_inst.first_level_    = json_inst["first_level"];
  params_inst.WTA_K_          = json_inst["WTA_K"];
  params_inst.score_type_ =
      static_cast<cv::ORB::ScoreType>(json_inst["score_type"]);
  params_inst.patch_size_     = json_inst["patch_size"];
  params_inst.fast_threshold_ = json_inst["fast_threshold"];
  params_inst.num_points_     = json_inst["num_points"];
  params_inst.show_debug_     = json_inst["show_debug"];

  params_inst.focal_ = 517.0f;
  params_inst.pp_    = cv::Point2f(318.6f, 255.3f);
  params_inst.dist_ =
      (cv::Mat_<float>(1, 5) << 0.2624, -0.9531, -0.0054, 0.0026, 1.1633);

  SPDLOG_INFO("");
  cv::Mat color0 = cv::imread(
      "/home/asharma/modern-rgbd-slam/apps/SlamPipeline/Data/rgb/000.png");
  cv::Mat color1 = cv::imread(
      "/home/asharma/modern-rgbd-slam/apps/SlamPipeline/Data/rgb/001.png");

  cv::Mat depth0 = cv::imread(
      "/home/asharma/modern-rgbd-slam/apps/SlamPipeline/Data/depth/000.png",
      cv::IMREAD_ANYDEPTH);
  cv::Mat depth1 = cv::imread(
      "/home/asharma/modern-rgbd-slam/apps/SlamPipeline/Data/depth/001.png",
      cv::IMREAD_ANYDEPTH);

  depth0.convertTo(depth0, CV_32FC1);
  depth0 = depth0 / 5000.0f;
  depth1.convertTo(depth1, CV_32FC1);
  depth1 = depth1 / 5000.0f;

  slam::Map map_inst;
  slam::Track track_inst;
  slam::Frame frame0(0, 0, color0, depth0);
  slam::Frame frame1(1, 0, color1, depth1);

  track_inst.Configure(params_inst);
  track_inst.Initialize(frame0, map_inst);

  return 0;
}
