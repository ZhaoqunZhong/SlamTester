//
// Created by 仲钊群 on 2022/10/10.
//

#include "TumRs_Pangolin_Example.h"
#include "glog/logging.h"
#include "yaml-cpp/yaml.h"

SlamTester::TumRsPangolinInput::TumRsPangolinInput(std::string &cam_config, std::string &ros_bag) {
    data_bag = ros_bag;

    YAML::Node camf = YAML::LoadFile(cam_config);

    if (camf["cam0"]["resolution"]) {
        LOG(INFO) << "Read video resolution.";
        auto video_size = camf["cam0"]["resolution"].as<std::vector<int>>();
        width = video_size[0];
        height = video_size[1];
    }

}

void SlamTester::PangolinFakeAlgorithm::feedMonoImg(double ts, cv::Mat mono) {
    for (auto & oi : output_interfaces) {
        oi->publishVideoImg(mono);
    }
}

void SlamTester::PangolinFakeAlgorithm::feedImu(double ts, Eigen::Vector3d acc, Eigen::Vector3d gyr) {
    for (auto & oi : output_interfaces) {
        oi->publishImuMsg(acc, gyr);
    }
}
