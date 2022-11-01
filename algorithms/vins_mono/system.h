//
// Created by zhong on 2021/11/5.
//
#ifndef VIO_SYSTEM_H
#define VIO_SYSTEM_H

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <fstream>
#include <condition_variable>
#include "vins_estimator/src/estimator.h"
#include "vins_estimator/src/parameters.h"
#include "feature_tracker/src/feature_tracker.h"
#include "feature_tracker/src/parameters.h"

/// Msg definition for sensor queue
struct IMU_MSG
{
    double header;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
};
typedef std::shared_ptr<IMU_MSG const> ImuConstPtr;

struct IMG_MSG {
    double header;
    vector<Vector3d> points;
    vector<int> id_of_point;
    vector<float> u_of_point;
    vector<float> v_of_point;
    vector<float> velocity_x_of_point;
    vector<float> velocity_y_of_point;
};
typedef std::shared_ptr <IMG_MSG const > ImgConstPtr;

class VinsAlgorithm;

class System
{
public:
    System(std::string sConfig_files, VinsAlgorithm *interface);

    ~System();

    void subImageData(double dStampSec, cv::Mat &img);

    void subImuData(double dStampSec, const Eigen::Vector3d &vGyr,
                    const Eigen::Vector3d &vAcc);


    // thread: visual-inertial odometry
    void process();
    std::thread vi_th_;
    bool bStart_backend, process_exited;
    std::condition_variable con;
private:
    VinsAlgorithm *algoInterface;

    FeatureTracker trackerData[feature_tracker::NUM_OF_CAM];



    //feature tracker
    std::vector<uchar> r_status;
    std::vector<float> r_err;
    // std::queue<ImageConstPtr> img_buf;


    double first_image_time;
    int pub_count = 1;
    bool first_image_flag = true;
    double last_image_time = 0;
    bool init_pub = 0;

    //estimator
    Estimator estimator;

    double current_time = -1;
    std::queue<ImuConstPtr> imu_buf;
    std::queue<ImgConstPtr> feature_buf;
    // std::queue<PointCloudConstPtr> relo_buf;
    int sum_of_wait = 0;

    std::mutex m_buf;
    std::mutex m_state;
    std::mutex i_buf;
    std::mutex m_estimator;

    double latest_time;
    Eigen::Vector3d tmp_P;
    Eigen::Quaterniond tmp_Q;
    Eigen::Vector3d tmp_V;
    Eigen::Vector3d tmp_Ba;
    Eigen::Vector3d tmp_Bg;
    Eigen::Vector3d acc_0;
    Eigen::Vector3d gyr_0;
    bool init_feature = 0;
    bool init_imu = 1;
    double last_imu_t = 0;
    // std::ofstream ofs_pose;
    // std::vector<Eigen::Vector3d> vPath_to_draw;

    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> getMeasurements();

};


#endif //VIO_SYSTEM_H
