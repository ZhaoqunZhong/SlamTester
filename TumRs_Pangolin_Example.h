//
// Created by 仲钊群 on 2022/10/10.
//

#ifndef SLAMTESTER_TUMRS_PANGOLIN_EXAMPLE_H
#define SLAMTESTER_TUMRS_PANGOLIN_EXAMPLE_H

#include "SlamInterface.h"
#include "PangolinViewer.h"

namespace SlamTester {

    class TumRsPangolinInput : public InputInterface {
    public:
        TumRsPangolinInput(std::string &cam_config, std::string &ros_bag);
        ~TumRsPangolinInput() override;

    };


    class PangolinFakeAlgorithm : public AlgorithmInterface {
    public:
        PangolinFakeAlgorithm() {}
        ~PangolinFakeAlgorithm() override {}

        void feedMonoImg(double ts, cv::Mat mono) override;
        void feedImu(double ts, Eigen::Vector3d acc, Eigen::Vector3d gyr) override;
    };
}

#endif //SLAMTESTER_TUMRS_PANGOLIN_EXAMPLE_H
