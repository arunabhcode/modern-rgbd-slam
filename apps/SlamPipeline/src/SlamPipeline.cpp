// Copyright 2018 Arunabh Sharma

#include <fstream>
#include <iostream>

#include "cxxopts.hpp"
#include "nlohmann/json.hpp"
#include "opencv2/opencv.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

#include "Room/Components/Frame.h"
#include "Room/Components/Pose.h"
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
    options.add_options()(
        "c,config", "Name of config file including path", cxxopts::value<std::string>());
    cxxopts::ParseResult result = options.parse(argc, argv);

    std::string config_file_str = result["c"].as<std::string>();
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
    config_file >> json_inst;

    cv::Mat color0 = cv::imread("/home/asharma/realsense-rgbd-slam/Data/1_color.png");
    cv::Mat color1 = cv::imread("/home/asharma/realsense-rgbd-slam/Data/2_color.png");

    cv::Mat depth0 =
        cv::imread("/home/asharma/realsense-rgbd-slam/Data/1_depth.png", cv::IMREAD_ANYDEPTH);
    cv::Mat depth1 =
        cv::imread("/home/asharma/realsense-rgbd-slam/Data/2_depth.png", cv::IMREAD_ANYDEPTH);

    depth0.convertTo(depth0, CV_32FC1);
    depth0 = depth0 / 5000.0f;
    depth1.convertTo(depth1, CV_32FC1);
    depth1 = depth1 / 5000.0f;
    // return 0;
    // Hardcode intrinsics for now

    float focal = 525.0f;
    Eigen::Vector2f pp(319.5f, 239.5f);
    room::Frame frame0(0, 0, color0, depth0, focal, pp);
    room::Frame frame1(1, 0, color1, depth1, focal, pp);

    int nfeatures                 = json_inst["nfeatures"];
    float scale_factor            = json_inst["scale_factor"];
    int nlevels                   = json_inst["nlevels"];
    int edge_threshold            = json_inst["edge_threshold"];
    int first_level               = json_inst["first_level"];
    int WTA_K                     = json_inst["WTA_K"];
    cv::ORB::ScoreType score_type = static_cast<cv::ORB::ScoreType>(json_inst["score_type"]);
    int patch_size                = json_inst["patch_size"];
    int fast_threshold            = json_inst["fast_threshold"];
    bool with_rotation            = json_inst["with_rotation"];
    bool with_scale               = json_inst["with_scale"];
    float threshold_factor        = json_inst["threshold_factor"];
    float num_points              = json_inst["num_points"];
    std::string show_debug        = json_inst["show_debug"];

    room::Feature feature_inst(show_debug,
                               nfeatures,
                               scale_factor,
                               nlevels,
                               edge_threshold,
                               first_level,
                               WTA_K,
                               score_type,
                               patch_size,
                               fast_threshold,
                               num_points);
    feature_inst.FeatureTrack(frame0, frame1, cv::Size(21, 21), 3);

    // room::Odometry odometry_inst(show_debug);
    // room::Pose out_pose = odometry_inst.PoseEstimationICP(frame0, frame1);

    // spdlog::info("Output rotation = \n {}", out_pose.m_orientation.toRotationMatrix());
    // spdlog::info("Output translation = \n {}", out_pose.m_position);
    // spdlog::info("Output Pose = \n {}", out_pose);

    return 0;
}
