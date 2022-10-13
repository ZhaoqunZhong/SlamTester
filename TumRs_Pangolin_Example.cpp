//
// Created by 仲钊群 on 2022/10/10.
//

#include "TumRs_Pangolin_Example.h"
#include "glog/logging.h"
#include "yaml-cpp/yaml.h"

namespace SlamTester {
    TumRsPangolinInput::TumRsPangolinInput(std::string &cam_config, std::string &ros_bag) {
        data_bag = ros_bag;

/*        YAML::Node camf = YAML::LoadFile(cam_config);
        if (!camf.IsScalar()) { // Check if yaml file is loaded.
            LOG(INFO) << "Read video resolution.";
            auto video_size = camf["cam0"]["resolution"].as<std::vector<int>>();
            orig_w = video_size[0];
            orig_h = video_size[1];
        }*/

        getUndistorterFromFile(cam_config, "", "");
    }

    TumRsPangolinInput::~TumRsPangolinInput() {

    }


    void PangolinFakeAlgorithm::feedMonoImg(double ts, cv::Mat mono) {
        cv::Mat processed_mat;
        input_interfaces[0]->undistortImg(mono, processed_mat);
        for (auto &oi: output_interfaces) {
            oi->publishVideoImg(mono);
            oi->publishProcessImg(processed_mat);
        }

    }

    void PangolinFakeAlgorithm::feedImu(double ts, Eigen::Vector3d acc, Eigen::Vector3d gyr) {
        for (auto &oi: output_interfaces) {
            oi->publishImuMsg(acc, gyr);
        }
    }
}

