//
// Created by 仲钊群 on 2022/10/19.
//

#pragma once

#include "SlamInterface.h"

namespace SlamTester {
    class TumRsDataset : public InputInterface {
    public:
        TumRsDataset(std::string &camConf, std::string &imuConf, std::string &ciExt, std::string &bag, std::string &gt,
                     bool rs = false);
        ~TumRsDataset() override = default;
    private:
        void loadGroundTruth(std::string &gt) override;
    };

    class EurocDataset : public InputInterface {
    public:
        EurocDataset(std::string &camConf, std::string &imuConf, std::string &ciExt, std::string &bag, std::string &gt);
        ~EurocDataset() override = default;
    private:
        void loadGroundTruth(std::string &gt) override;
    };
}


