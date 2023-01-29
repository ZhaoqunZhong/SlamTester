//
// Created by 仲钊群 on 2022/10/10.
//

#include <sys/time.h>
#include "Pangolin_Algo_Example.h"
#include "glog/logging.h"

namespace SlamTester {
    void PangolinFakeAlgorithm::feedMonoImg(double ts, cv::Mat mono) {
        std::vector<cv::Point2f> features;
        // mono.convertTo(mono, CV_8U, 1.0/256);
        cv::goodFeaturesToTrack(mono, features, 150, 0.01, 30);
        cv::Mat show_img;
        cv::cvtColor(mono, show_img, cv::COLOR_GRAY2RGB);
        for (unsigned int j = 0; j < features.size(); j++) {
            double len = 0.5;
            cv::circle(show_img, features[j], 2,
                       cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
        }
        for (auto &oi: output_interfaces) {
            oi->publishProcessImg(show_img);
        }

    }

    void PangolinFakeAlgorithm::feedImu(double ts, Eigen::Vector3d acc, Eigen::Vector3d gyr) {
        SlamTester::AlgorithmInterface::feedImu(ts, acc, gyr);
    }
}

