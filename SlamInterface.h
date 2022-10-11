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



    // A certain slam algorithm 'has' an InputInterface.
    // So use member class ptr.
    class InputInterface {
    public:

        InputInterface() {}
        virtual ~InputInterface() {}


        // Paras that read from config files.
        Eigen::Matrix3d cam_k;
        Eigen::VectorXd cam_distort;
        double imu_na, imu_ra, imu_ng, imu_rg;
        Eigen::Matrix4d camToImu;
        uint width=640, height=480;

        std::string data_bag;
        std::string monoImg_topic = "/cam0/image_raw";
        std::string imu_topic = "/imu0";
        std::string acc_topic = "/acc0";
        std::string gyr_topic = "/gyr0";
        std::vector<std::string> bag_topics {monoImg_topic, imu_topic, acc_topic, gyr_topic};

    private:

    };


    // A certain algorithm should contain both input and output interfaces.
    class AlgorithmInterface {
    public:
        AlgorithmInterface() {}
        virtual ~AlgorithmInterface() {}

        std::unique_ptr<InputInterface> input_interface;
        std::vector<std::shared_ptr<OutputInterface>> output_interfaces;

        virtual void feedMonoImg(double ts, cv::Mat mono) {}
        virtual void feedImu(double ts, Eigen::Vector3d acc, Eigen::Vector3d gyr) {}
    };
}

#endif //SLAMTESTER_SLAMINTERFACE_H
