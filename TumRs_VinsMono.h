//
// Created by 仲钊群 on 2022/10/9.
//

#ifndef SLAMTESTER_TUMRS_VINSMONO_H
#define SLAMTESTER_TUMRS_VINSMONO_H

#include "SlamInterface.h"

namespace SlamTester {

    class TumRsPangolin : public InputInterface {
    public:
        TumRsPangolin(std::string &imu_config, std::string &cam_config, std::string &ros_bag) : data_bag(ros_bag) {}
        ~TumRsPangolin() override;


        std::string data_bag;
        std::string monoImg_topic = "/cam0/image_raw";
        std::string imu_topic = "/imu0";
        std::string acc_topic = "/acc0";
        std::string gyr_topic = "/gyr0";
        std::vector<std::string> bag_topics {monoImg_topic, imu_topic, acc_topic, gyr_topic};
    };
}

#endif //SLAMTESTER_TUMRS_VINSMONO_H
