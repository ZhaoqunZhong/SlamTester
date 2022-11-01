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

        loadGroundTruth(gt);
    }

    //gt file in '# timestamp[ns]	tx	ty	tz	qw	qx	qy	qz' format
    void TumRsDataset::loadGroundTruth(std::string &gt) {
        LOG(INFO) << "Derived loadGroundTruth method called for TUM RS dataset.";
        if (gt.empty())
            return;

        std::ifstream gtf(gt);
        if (!gtf.good()) {
            gtf.close();
            LOG(ERROR) << "Failed to find ground truth file.";
            return;
        }

        // Loop through each line of this file
        std::string current_line;
        while (std::getline(gtf, current_line)) {

            // Skip if we start with a comment
            if (!current_line.find("#"))
                continue;

            // Loop variables
            int i = 0;
            std::istringstream s(current_line);
            std::string field;
            Eigen::Matrix<double, 20, 1> data;

            char dlm;
            if (std::filesystem::path(gt).extension() == ".csv")
                dlm = ',';
            else
                dlm = ' ';
            // Loop through this line (timestamp(s) tx ty tz qx qy qz qw Pr11 Pr12 Pr13 Pr22 Pr23 Pr33 Pt11 Pt12 Pt13 Pt22 Pt23 Pt33)
            while (std::getline(s, field, dlm)) {
                // Skip if empty
                if (field.empty() || i >= data.rows())
                    continue;
                // save the data to our vector
                data(i) = std::atof(field.c_str());
                i++;
            }

            // Only a valid line if we have all the parameters
            if (i >= 20) {
                // time and pose
                gt_times_s.push_back(data(0) / 1e9);
                gt_poses.emplace_back(data(1), data(2), data(3), data(5),
                                      data(6), data(7), data(4));
                // covariance values
/*                Eigen::Matrix3d c_ori, c_pos;
                c_ori << data(8), data(9), data(10), data(9), data(11), data(12), data(10), data(12), data(13);
                c_pos << data(14), data(15), data(16), data(15), data(17), data(18), data(16), data(18), data(19);
                c_ori = 0.5 * (c_ori + c_ori.transpose());
                c_pos = 0.5 * (c_pos + c_pos.transpose());
                cov_ori.push_back(c_ori);
                cov_pos.push_back(c_pos);*/
            } else if (i >= 8) {
                gt_times_s.push_back(data(0) / 1e9);
                gt_poses.emplace_back(data(1), data(2), data(3), data(5),
                                      data(6), data(7), data(4));
            }
        }

        // Finally close the file
        gtf.close();

        // Error if we don't have any data
        if (gt_times_s.empty()) {
            // PRINT_ERROR(RED "[LOAD]: Could not parse any data from the file!!\n" RESET);
            // PRINT_ERROR(RED "[LOAD]: %s\n" RESET, path_traj.c_str());
            LOG(ERROR) << "Could not parse any data from gt file!!";
            std::exit(EXIT_FAILURE);
        }

        // Assert that they are all equal
        if (gt_times_s.size() != gt_poses.size()) {
            // PRINT_ERROR(RED "[LOAD]: Parsing error, pose and timestamps do not match!!\n" RESET);
            // PRINT_ERROR(RED "[LOAD]: %s\n" RESET, path_traj.c_str());
            LOG(ERROR) << "Parsing error, gt poses and timestamps do not match!!";
            std::exit(EXIT_FAILURE);
        }
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

        loadGroundTruth(gt);
    }

    void EurocDataset::loadGroundTruth(std::string &gt) {
        //gt file in '# timestamp[ns]	tx	ty	tz	qw	qx	qy	qz' format
        LOG(INFO) << "Derived loadGroundTruth method called for Euroc dataset.";
        if (gt.empty())
            return;

        std::ifstream gtf(gt);
        if (!gtf.good()) {
            gtf.close();
            LOG(ERROR) << "Failed to find ground truth file.";
            return;
        }

        // Loop through each line of this file
        std::string current_line;
        while (std::getline(gtf, current_line)) {

            // Skip if we start with a comment
            if (!current_line.find("#"))
                continue;

            // Loop variables
            int i = 0;
            std::istringstream s(current_line);
            std::string field;
            Eigen::Matrix<double, 20, 1> data;

            char dlm;
            if (std::filesystem::path(gt).extension() == ".csv")
                dlm = ',';
            else
                dlm = ' ';
            // Loop through this line (timestamp(s) tx ty tz qx qy qz qw Pr11 Pr12 Pr13 Pr22 Pr23 Pr33 Pt11 Pt12 Pt13 Pt22 Pt23 Pt33)
            while (std::getline(s, field, dlm)) {
                // Skip if empty
                if (field.empty() || i >= data.rows())
                    continue;
                // save the data to our vector
                data(i) = std::atof(field.c_str());
                i++;
            }

            // Only a valid line if we have all the parameters
            if (i >= 20) {
                // time and pose
                gt_times_s.push_back(data(0) / 1e9);
                gt_poses.emplace_back(data(1), data(2), data(3), data(5),
                                      data(6), data(7), data(4));
                // covariance values
/*                Eigen::Matrix3d c_ori, c_pos;
                c_ori << data(8), data(9), data(10), data(9), data(11), data(12), data(10), data(12), data(13);
                c_pos << data(14), data(15), data(16), data(15), data(17), data(18), data(16), data(18), data(19);
                c_ori = 0.5 * (c_ori + c_ori.transpose());
                c_pos = 0.5 * (c_pos + c_pos.transpose());
                cov_ori.push_back(c_ori);
                cov_pos.push_back(c_pos);*/
            } else if (i >= 8) {
                gt_times_s.push_back(data(0) / 1e9);
                gt_poses.emplace_back(data(1), data(2), data(3), data(5),
                                      data(6), data(7), data(4));
            }
        }

        // Finally close the file
        gtf.close();

        // Error if we don't have any data
        if (gt_times_s.empty()) {
            // PRINT_ERROR(RED "[LOAD]: Could not parse any data from the file!!\n" RESET);
            // PRINT_ERROR(RED "[LOAD]: %s\n" RESET, path_traj.c_str());
            LOG(ERROR) << "Could not parse any data from gt file!!";
            std::exit(EXIT_FAILURE);
        }

        // Assert that they are all equal
        if (gt_times_s.size() != gt_poses.size()) {
            // PRINT_ERROR(RED "[LOAD]: Parsing error, pose and timestamps do not match!!\n" RESET);
            // PRINT_ERROR(RED "[LOAD]: %s\n" RESET, path_traj.c_str());
            LOG(ERROR) << "Parsing error, gt poses and timestamps do not match!!";
            std::exit(EXIT_FAILURE);
        }

    }
}
