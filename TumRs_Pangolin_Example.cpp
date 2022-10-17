//
// Created by 仲钊群 on 2022/10/10.
//

#include <sys/time.h>
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
        monoImg_topic = "/cam1/image_raw";
        bag_topics = {monoImg_topic, imu_topic, acc_topic, gyr_topic};

        getUndistorterFromFile(cam_config, "", "");
    }



    void PangolinFakeAlgorithm::feedMonoImg(double ts, cv::Mat mono) {
        cv::Mat processed_mat;
/*        struct timeval time0;
        gettimeofday(&time0, NULL);*/
        input_interfaces[0]->undistortImg(mono, processed_mat);
/*        struct timeval time1;
        gettimeofday(&time1, NULL);
        LOG_FIRST_N(INFO, 100) << "undistortImg function costs " <<
                              (time1.tv_sec - time0.tv_sec) * 1000.0f + (time1.tv_usec - time0.tv_usec) / 1000.0f;*/
        std::vector<cv::Point2f> features;
        processed_mat.convertTo(processed_mat, CV_8U, 1.0/256);
        cv::goodFeaturesToTrack(processed_mat, features, 150, 0.01, 30);
        cv::Mat show_img;
        cv::cvtColor(processed_mat, show_img, cv::COLOR_GRAY2RGB);
        for (unsigned int j = 0; j < features.size(); j++) {
            double len = 0.5;
            cv::circle(show_img, features[j], 2,
                       cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
        }
        for (auto &oi: output_interfaces) {
            oi->publishVideoImg(mono);
            oi->publishProcessImg(show_img);
        }

    }

    void PangolinFakeAlgorithm::feedImu(double ts, Eigen::Vector3d acc, Eigen::Vector3d gyr) {
        for (auto &oi: output_interfaces) {
            oi->publishImuMsg(acc, gyr);
        }
    }
}

