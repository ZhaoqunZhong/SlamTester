//
// Created by 仲钊群 on 2022/10/8.
//

#ifndef SLAMTESTER_SLAMINTERFACE_H
#define SLAMTESTER_SLAMINTERFACE_H

#include "opencv2/opencv.hpp"
#include "Eigen/Core"

namespace SlamTester {
    // PangolinViewer 'is' an OutputInterface.
    // So use inheritance.
    class OutputInterface {
    public:
        OutputInterface() {}
        virtual ~OutputInterface() {}
        
        virtual void publishCamPose(Eigen::Matrix4d & cam_pose) {}
        virtual void publishImuPose(Eigen::Matrix4d & imu_pose) {}

        virtual void publishVideoImg(cv::Mat video_img) {}
        virtual void publishProcessImg(cv::Mat process_img) {}
        virtual void publishImuMsg(Eigen::Vector3d acc, Eigen::Vector3d gyr) {}
    };


    enum DistortModel{
        PinHole,
        RadTan,
        EquiDistant,
        KannalaBrandt,
        FOV
    };
    // A certain slam algorithm 'has' an InputInterface.
    // So use member class ptr.
    // EASY to support multiple cameras.
    class InputInterface {
    public:

        InputInterface(std::string &cam_conf, std::string &imu_conf, std::string &ci_ext,
                       std::string &bag, std::string &gt) : cam_config(cam_conf), imu_config(imu_conf),
                       ci_extrinsic(ci_ext), data_bag(bag), ground_truth(gt) {}
        virtual ~InputInterface() {}

        // Paras that read from config files.
        cv::Matx33d cam_k;
        std::vector<double> cam_distort;
        DistortModel distort_model;
        double imu_na, imu_ra, imu_ng, imu_rg;
        Eigen::Matrix4d camToImu;
        double timeshift_cam;
        uint orig_w, orig_h;

        // Offline data (No use for online running.)
        std::string data_bag;
        std::string monoImg_topic = "/cam0/image_raw";
        std::string imu_topic = "/imu0";
        std::string acc_topic = "/acc0";
        std::string gyr_topic = "/gyr0";
        std::vector<std::string> bag_topics;
        std::string cam_config;
        std::string imu_config;
        std::string ci_extrinsic;
        std::string ground_truth;
        std::vector<Eigen::Matrix<double, 7, 1>> gt_poses; //x, y, z, qx, qy, qz, qw
        std::vector<double> gt_times_s;

        // unDistortion
        uint inner_w, inner_h;
        cv::Mat remapX, remapY;
        cv::Matx33d inner_cam_k;
        void undistortImg(cv::Mat in, cv::Mat &out);

    protected:
        void getUndistorterFromFile(std::string configFilename, std::string gammaFilename, std::string vignetteFilename);
        virtual void loadGroundTruth(std::string &gt_file);
    };


    // A certain algorithm should contain both input and output interfaces.
    class AlgorithmInterface {
    public:
        AlgorithmInterface() {}
        virtual ~AlgorithmInterface() {}

        // A certain algorithm can have multiple input and output interfaces.
        // Such as multiple camera sensors, and different form of output wrappers.
        std::vector<std::shared_ptr<InputInterface>> input_interfaces;
        std::vector<std::shared_ptr<OutputInterface>> output_interfaces;

        virtual void feedMonoImg(double ts, cv::Mat mono) {}
        virtual void feedImu(double ts, Eigen::Vector3d acc, Eigen::Vector3d gyr) {}

        virtual void start() {}
        virtual void stop() {}
    };
}

#endif //SLAMTESTER_SLAMINTERFACE_H
