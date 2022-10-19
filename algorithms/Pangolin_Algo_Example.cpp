//
// Created by 仲钊群 on 2022/10/10.
//

#include <sys/time.h>
#include "Pangolin_Algo_Example.h"
#include "glog/logging.h"

namespace SlamTester {
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

