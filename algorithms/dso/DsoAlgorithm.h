//
// Created by 仲钊群 on 2022/10/31.
//

#ifndef SLAMTESTER_DSOALGORITHM_H
#define SLAMTESTER_DSOALGORITHM_H

#include "../../SlamInterface.h"

namespace dso {
    class FullSystem;
}

class DsoAlgorithm : public SlamTester::AlgorithmInterface {
public:
    DsoAlgorithm() {}
    ~DsoAlgorithm() {}
    void start() override;
    void feedMonoImg(double ts, cv::Mat mono) override;

protected:
    std::shared_ptr<dso::FullSystem> dso_algo;
};

#endif //SLAMTESTER_DSOALGORITHM_H
