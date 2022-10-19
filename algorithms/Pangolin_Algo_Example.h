//
// Created by 仲钊群 on 2022/10/10.
//

#ifndef SLAMTESTER_PANGOLIN_ALGO_EXAMPLE_H
#define SLAMTESTER_PANGOLIN_ALGO_EXAMPLE_H

#include "../SlamInterface.h"

namespace SlamTester {

    class PangolinFakeAlgorithm : public AlgorithmInterface {
    public:
        PangolinFakeAlgorithm() {}
        ~PangolinFakeAlgorithm() override = default;

        void feedMonoImg(double ts, cv::Mat mono) override;
        void feedImu(double ts, Eigen::Vector3d acc, Eigen::Vector3d gyr) override;
    };
}

#endif //SLAMTESTER_PANGOLIN_ALGO_EXAMPLE_H
