//
// Created by 仲钊群 on 2022/10/19.
//

#include <fstream>
#include "DataSets.h"
#include "yaml-cpp/yaml.h"
#include "glog/logging.h"

namespace SlamTester {
    TumRsDataset::TumRsDataset(std::string &camConf, std::string &imuConf, std::string &ciExt,
                               std::string &bag, std::string &gt, bool rs)
            : InputInterface(camConf, imuConf, ciExt, bag, gt) {
        /*
        YAML::Node imuf = YAML::LoadFile(imu_config);
        if (imuf.IsScalar()) {
            LOG(ERROR) << "Failed to read imu config file.";
            exit(1);
        } else {
            LOG(INFO) << "Read imu config file.";
            imu_na = imuf["accelerometer_noise_density"].as<double>();
            imu_ra = imuf["accelerometer_random_walk"].as<double>();
            imu_ng = imuf["gyroscope_noise_density"].as<double>();
            imu_rg = imuf["gyroscope_random_walk"].as<double>();
            LOG(INFO) << "imu_na: " << imu_na;
            LOG(INFO) << "imu_ra: " << imu_ra;
            LOG(INFO) << "imu_ng: " << imu_ng;
            LOG(INFO) << "imu_rg: " << imu_rg;
            imu_topic = imuf["rostopic"].as<std::string>();
            LOG(INFO) << "imu ros topic: " << imu_topic;
        }

        YAML::Node cif = YAML::LoadFile(ci_extrinsic);
        if (cif.IsScalar()) {
            LOG(ERROR) << "Failed to read cam_imu calibration file.";
            exit(1);
        } else {
            LOG(INFO) << "Read cam_imu calibration file.";
            std::string cam = "cam0";// gs cam
            monoImg_topic = cif[cam]["rostopic"].as<std::string>();
            LOG(INFO) << "mono image rostopic: " << monoImg_topic;
            camToImu.row(0) = Eigen::RowVector4d(cif[cam]["T_cam_imu"][0].as<std::vector<double>>().data());
            camToImu.row(1) = Eigen::RowVector4d(cif[cam]["T_cam_imu"][1].as<std::vector<double>>().data());
            camToImu.row(2) = Eigen::RowVector4d(cif[cam]["T_cam_imu"][2].as<std::vector<double>>().data());
            camToImu.row(3) = Eigen::RowVector4d(cif[cam]["T_cam_imu"][3].as<std::vector<double>>().data());
            LOG(INFO) << "cam to imu extrinsic: \n" << camToImu;
            // LOG(INFO) << "imu to cam extrinsic: \n" << camToImu.inverse();
            timeshift_cam = cif[cam]["timeshift_cam_imu"].as<double>();
            LOG(INFO) << "timeshift cam to imu: " << timeshift_cam;
            orig_w = inner_w = cif[cam]["resolution"][0].as<double>();
            orig_h = inner_h = cif[cam]["resolution"][1].as<double>();
            LOG(INFO) << "orig_w, orig_h: " << orig_w << "," << orig_h;
            LOG(INFO) << "inner_w, inner_h: " << inner_w << "," << inner_h;
        }
        */

        imu_topic = "/imu0";
        YAML::Node cif = YAML::LoadFile(ci_extrinsic);
        if (rs) {
            monoImg_topic = "/cam1/image_raw";
            camToImu.row(0) = Eigen::RowVector4d(cif["cam1"]["T_cam_imu"][0].as<std::vector<double>>().data());
            camToImu.row(1) = Eigen::RowVector4d(cif["cam1"]["T_cam_imu"][1].as<std::vector<double>>().data());
            camToImu.row(2) = Eigen::RowVector4d(cif["cam1"]["T_cam_imu"][2].as<std::vector<double>>().data());
            camToImu.row(3) = Eigen::RowVector4d(cif["cam1"]["T_cam_imu"][3].as<std::vector<double>>().data());
        } else {
            monoImg_topic = "/cam0/image_raw";
            camToImu.row(0) = Eigen::RowVector4d(cif["cam0"]["T_cam_imu"][0].as<std::vector<double>>().data());
            camToImu.row(1) = Eigen::RowVector4d(cif["cam0"]["T_cam_imu"][1].as<std::vector<double>>().data());
            camToImu.row(2) = Eigen::RowVector4d(cif["cam0"]["T_cam_imu"][2].as<std::vector<double>>().data());
            camToImu.row(3) = Eigen::RowVector4d(cif["cam0"]["T_cam_imu"][3].as<std::vector<double>>().data());
        }
        LOG(INFO) << "cam to imu extrinsic: \n" << camToImu;
        LOG(INFO) << "imu to cam extrinsic: \n" << camToImu.inverse();
        bag_topics = {monoImg_topic, imu_topic};
/*        LOG(INFO) << "Used rosbag topics: ";
        for (auto &topic : bag_topics) {
            LOG(INFO) << topic;
        }*/

        getUndistorterFromFile(cam_config, "", "");
        LOG(INFO) << "orig_w, orig_h: " << orig_w << "," << orig_h;
        LOG(INFO) << "inner_w, inner_h: " << inner_w << "," << inner_h;

        loadGroundTruthEuroc(gt);
    }

    EurocDataset::EurocDataset(std::string &camConf, std::string &imuConf, std::string &ciExt,
                               std::string &bag, std::string &gt)
            : InputInterface(camConf, imuConf, ciExt, bag, gt) {
        imu_topic = "/imu0";
        monoImg_topic = "/cam0/image_raw";
        bag_topics = {monoImg_topic, imu_topic};

        getUndistorterFromFile(cam_config, "", "");
        LOG(INFO) << "orig_w, orig_h: " << orig_w << "," << orig_h;
        LOG(INFO) << "inner_w, inner_h: " << inner_w << "," << inner_h;

        YAML::Node cif = YAML::LoadFile(ci_extrinsic);
        Eigen::Matrix4d imu_to_cam = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>(cif["T_BS"]["data"].as<std::vector<double>>().data());
        camToImu = imu_to_cam.inverse();

        LOG(INFO) << "cam to imu extrinsic: \n" << camToImu;
        LOG(INFO) << "imu to cam extrinsic: \n" << imu_to_cam;

        loadGroundTruthEuroc(gt);
    }

}
