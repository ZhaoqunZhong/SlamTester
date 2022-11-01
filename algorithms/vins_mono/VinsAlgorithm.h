//
// Created by 仲钊群 on 2022/11/1.
//

#ifndef SLAMTESTER_VINSALGORITHM_H
#define SLAMTESTER_VINSALGORITHM_H

#include "../../SlamInterface.h"

class System;

class VinsAlgorithm : public SlamTester::AlgorithmInterface {
public:
    VinsAlgorithm(std::string &config);
    ~VinsAlgorithm() override {}
    void start() override;
    void stop() override;
    void feedMonoImg(double ts, cv::Mat mono) override;
    void feedImu(double ts, Eigen::Vector3d acc, Eigen::Vector3d gyr) override;
private:
    std::shared_ptr<System> vins_algo;
};

#endif //SLAMTESTER_VINSALGORITHM_H
