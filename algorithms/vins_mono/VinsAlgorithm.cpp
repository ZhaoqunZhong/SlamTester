//
// Created by 仲钊群 on 2022/11/1.
//

#include "VinsAlgorithm.h"
#include "system.h"
#include <memory>

VinsAlgorithm::VinsAlgorithm(string &config) {
    vins_algo = std::make_unique<System>(config, this);
}


void VinsAlgorithm::start() {
    vins_algo->vi_th_ = std::thread(&System::process, vins_algo);
    vins_algo->vi_th_.detach();
}

void VinsAlgorithm::stop() {
    vins_algo->bStart_backend = false;
    vins_algo->con.notify_one();
    while (!vins_algo->process_exited) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    vins_algo.reset();
}

void VinsAlgorithm::feedMonoImg(double ts, cv::Mat mono) {
    SlamTester::AlgorithmInterface::feedMonoImg(ts, mono);

    if (mono.depth() == CV_16U)
        mono.convertTo(mono, CV_8U, 1.0/256);
    vins_algo->subImageData(ts, mono);
}

void VinsAlgorithm::feedImu(double ts, Eigen::Vector3d acc, Eigen::Vector3d gyr) {
    SlamTester::AlgorithmInterface::feedImu(ts, acc, gyr);

    vins_algo->subImuData(ts, gyr, acc);
}



