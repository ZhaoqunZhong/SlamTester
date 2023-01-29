//
// Created by 仲钊群 on 2022/10/10.
//

#include <iostream>
#include <sys/time.h>
#include "PangolinViewer.h"

#include "RosbagStorage/rosbag/view.h"
#include "MessageType/sensor_msgs/Image.h"
#include "MessageType/sensor_msgs/CompressedImage.h"
#include "MessageType/sensor_msgs/Imu.h"
#include "MessageType/geometry_msgs/Vector3Stamped.h"
#include "cv_bridge_simple.h"

#include "DataSets.h"
#include "algorithms/Pangolin_Algo_Example.h"
#include "algorithms/vins_mono/VinsAlgorithm.h"
#include "algorithms/dso/DsoAlgorithm.h"

DEFINE_string(algorithm, "", "Choose one of the following algorithms {example_undistort, vins_mono, dso}");
DEFINE_string(dataset, "", "Choose one of the following datasets {euroc, tum_rs}");
DEFINE_string(algoConfig, "", "Path to algorithm config file.");
DEFINE_string(camConfig, "", "Path to internal camera config file.");
DEFINE_string(imuConfig, "", "Path to imu noise parameters file.");
DEFINE_string(ciExtrinsic, "", "Path to camera to imu extrinsic file.");
DEFINE_string(dataBag, "", "Path to rosbag file.");
DEFINE_string(groundTruth, "", "Path to ground truth file.");
DEFINE_bool(rs_cam, false, "Choose rolling shutter camera data if available.");
DEFINE_bool(resizeAndUndistort, false, "Resize the rgb resolution and undistort before feeding to algorithm.");
DEFINE_string(gammaFile, "", "Path to camera photometric gamma file.");
DEFINE_string(vignetteImage, "", "Path to camera photometric vignette image.");
DEFINE_bool(skipCamMsgIfLag, false, "Skip image if algorithm processing lags behind.");
DEFINE_bool(showOrigCamStream, false, "");

// Finite automata to sync acc and gyr.
enum Input {acc,gyr} cur_input;
enum State {WAIT_FOR_MSG, ACC, ACC_GYR, ACC_GYR_ACC, ACC_GYRs, ACC_GYR_ACCs} cur_state;
struct acc_msg {
    acc_msg() {}
    acc_msg(double _ts, double _ax, double _ay, double _az) :
            ts(_ts), ax(_ax), ay(_ay), az(_az) {}
    double ts;
    double ax;
    double ay;
    double az;
};
struct gyr_msg {
    gyr_msg() {}
    gyr_msg(double _ts, double _rx, double _ry, double _rz) :
            ts(_ts), rx(_rx), ry(_ry), rz(_rz) {}
    double ts;
    double rx;
    double ry;
    double rz;
};
struct imu_msg {
    imu_msg() {}
    double ts;
    acc_msg acc_part;
    gyr_msg gyro_part;
};
std::vector<gyr_msg> gyro_cache_;
std::vector<acc_msg> acc_cache_;
void constructImuInterpolateAcc(SlamTester::AlgorithmInterface *algorithmInterface) {
    switch (cur_state) {
        case WAIT_FOR_MSG:
            if (cur_input == acc) {
                cur_state = ACC;
            } else {
                gyro_cache_.clear();
            }
            break;
        case ACC:
            if (cur_input == acc) {
                acc_cache_.erase(acc_cache_.begin());
            } else {
                if (gyro_cache_.front().ts <= acc_cache_.front().ts) {
                    gyro_cache_.clear();
                } else {
                    cur_state = ACC_GYR;
                }
            }
            break;
        case ACC_GYR:
            if (cur_input == acc) {
                if (acc_cache_.back().ts <= gyro_cache_.front().ts) {
                    acc_cache_.erase(acc_cache_.begin());
                } else {
                    imu_msg imu;
                    imu.ts = gyro_cache_.front().ts;
                    imu.gyro_part = gyro_cache_.front();
                    double factor = (imu.ts - acc_cache_.front().ts)/(acc_cache_.back().ts - acc_cache_.front().ts);
                    imu.acc_part = {imu.ts, acc_cache_.front().ax * (1-factor) + acc_cache_.back().ax *factor,
                                    acc_cache_.front().ay * (1-factor) + acc_cache_.back().ay *factor,
                                    acc_cache_.front().az * (1-factor) + acc_cache_.back().az *factor};
                    algorithmInterface->feedImu(imu.ts, Eigen::Vector3d{imu.acc_part.ax, imu.acc_part.ay, imu.acc_part.az},
                                             Eigen::Vector3d{imu.gyro_part.rx, imu.gyro_part.ry, imu.gyro_part.rz});
                    cur_state = ACC_GYR_ACC;
                }
            } else {
                cur_state = ACC_GYRs;
            }
            break;
        case ACC_GYRs:
            if (cur_input == acc) {
                if (acc_cache_.back().ts <= gyro_cache_.front().ts) {
                    acc_cache_.erase(acc_cache_.begin());
                } else if (acc_cache_.back().ts > gyro_cache_.front().ts && acc_cache_.back().ts < gyro_cache_.back().ts) {
                    int i;
                    for (i = 0; i < gyro_cache_.size(); i++) {
                        if (gyro_cache_[i].ts < acc_cache_.back().ts) {
                            imu_msg imu;
                            imu.ts = gyro_cache_[i].ts;
                            imu.gyro_part = gyro_cache_[i];
                            double factor = (imu.ts - acc_cache_.front().ts)/(acc_cache_.back().ts - acc_cache_.front().ts);
                            imu.acc_part = {imu.ts, acc_cache_.front().ax * (1-factor) + acc_cache_.back().ax *factor,
                                            acc_cache_.front().ay * (1-factor) + acc_cache_.back().ay *factor,
                                            acc_cache_.front().az * (1-factor) + acc_cache_.back().az *factor};
                            algorithmInterface->feedImu(imu.ts, Eigen::Vector3d{imu.acc_part.ax, imu.acc_part.ay, imu.acc_part.az},
                                                     Eigen::Vector3d{imu.gyro_part.rx, imu.gyro_part.ry, imu.gyro_part.rz});
                        } else {
                            break;
                        }
                    }
                    acc_cache_.erase(acc_cache_.begin());
                    gyro_cache_.erase(gyro_cache_.begin(), gyro_cache_.begin() + i);
                } else {
                    for (gyr_msg & gyro : gyro_cache_) {
                        imu_msg imu;
                        imu.ts = gyro.ts;
                        imu.gyro_part = gyro;
                        double factor = (imu.ts - acc_cache_.front().ts)/(acc_cache_.back().ts - acc_cache_.front().ts);
                        imu.acc_part = {imu.ts, acc_cache_.front().ax * (1-factor) + acc_cache_.back().ax *factor,
                                        acc_cache_.front().ay * (1-factor) + acc_cache_.back().ay *factor,
                                        acc_cache_.front().az * (1-factor) + acc_cache_.back().az *factor};
                        algorithmInterface->feedImu(imu.ts, Eigen::Vector3d{imu.acc_part.ax, imu.acc_part.ay, imu.acc_part.az},
                                                 Eigen::Vector3d{imu.gyro_part.rx, imu.gyro_part.ry, imu.gyro_part.rz});
                    }
                    gyro_cache_.erase(gyro_cache_.begin(), gyro_cache_.begin() + gyro_cache_.size() - 1);
                    cur_state = ACC_GYR_ACC;
                }
            } else {}
            break;
        case ACC_GYR_ACC:
            if (cur_input == acc) {
                cur_state = ACC_GYR_ACCs;
            } else {
                if (gyro_cache_.back().ts > acc_cache_.back().ts) {
                    acc_cache_.erase(acc_cache_.begin());
                    gyro_cache_.erase(gyro_cache_.begin());
                    cur_state = ACC_GYR;
                } else {
                    imu_msg imu;
                    imu.ts = gyro_cache_.back().ts;
                    imu.gyro_part = gyro_cache_.back();
                    double factor = (imu.ts - acc_cache_.front().ts)/(acc_cache_.back().ts - acc_cache_.front().ts);
                    imu.acc_part = {imu.ts, acc_cache_.front().ax * (1-factor) + acc_cache_.back().ax *factor,
                                    acc_cache_.front().ay * (1-factor) + acc_cache_.back().ay *factor,
                                    acc_cache_.front().az * (1-factor) + acc_cache_.back().az *factor};
                    algorithmInterface->feedImu(imu.ts, Eigen::Vector3d{imu.acc_part.ax, imu.acc_part.ay, imu.acc_part.az},
                                             Eigen::Vector3d{imu.gyro_part.rx, imu.gyro_part.ry, imu.gyro_part.rz});
                    gyro_cache_.erase(gyro_cache_.begin());
                }
            }
            break;
        case ACC_GYR_ACCs:
            if (cur_input == acc) {

            } else {
                if (gyro_cache_.back().ts <= acc_cache_[1].ts) {
                    imu_msg imu;
                    imu.ts = gyro_cache_.back().ts;
                    imu.gyro_part = gyro_cache_.back();
                    double factor = (imu.ts - acc_cache_.front().ts)/(acc_cache_.back().ts - acc_cache_.front().ts);
                    imu.acc_part = {imu.ts, acc_cache_.front().ax * (1-factor) + acc_cache_.back().ax *factor,
                                    acc_cache_.front().ay * (1-factor) + acc_cache_.back().ay *factor,
                                    acc_cache_.front().az * (1-factor) + acc_cache_.back().az *factor};
                    algorithmInterface->feedImu(imu.ts, Eigen::Vector3d{imu.acc_part.ax, imu.acc_part.ay, imu.acc_part.az},
                                             Eigen::Vector3d{imu.gyro_part.rx, imu.gyro_part.ry, imu.gyro_part.rz});
                    gyro_cache_.erase(gyro_cache_.begin());
                } else if (gyro_cache_.back().ts >= acc_cache_.back().ts) {
                    gyro_cache_.erase(gyro_cache_.begin());
                    acc_cache_.erase(acc_cache_.begin(), acc_cache_.begin() + acc_cache_.size() - 1);
                    cur_state = ACC_GYR;
                } else {
                    int i;
                    for (i = 0; i < acc_cache_.size(); i++) {
                        if (acc_cache_[i].ts >= gyro_cache_.back().ts)
                            break;
                    }
                    imu_msg imu;
                    imu.ts = gyro_cache_.back().ts;
                    imu.gyro_part = gyro_cache_.back();
                    double factor = (imu.ts - acc_cache_[i-1].ts)/(acc_cache_[i].ts - acc_cache_[i-1].ts);
                    imu.acc_part = {imu.ts, acc_cache_[i-1].ax * (1-factor) + acc_cache_[i].ax *factor,
                                    acc_cache_[i-1].ay * (1-factor) + acc_cache_[i].ay *factor,
                                    acc_cache_[i-1].az * (1-factor) + acc_cache_[i].az *factor};
                    algorithmInterface->feedImu(imu.ts, Eigen::Vector3d{imu.acc_part.ax, imu.acc_part.ay, imu.acc_part.az},
                                             Eigen::Vector3d{imu.gyro_part.rx, imu.gyro_part.ry, imu.gyro_part.rz});
                    gyro_cache_.erase(gyro_cache_.begin());
                    acc_cache_.erase(acc_cache_.begin(), acc_cache_.begin() + i-1);
                }
            }
            break;
    }
}
void feedAcc(double ts, Eigen::Vector3d acc_msg, SlamTester::AlgorithmInterface *algorithmInterface) {
    acc_cache_.emplace_back(ts, acc_msg.x(), acc_msg.y(), acc_msg.z());
    cur_input = acc;
    constructImuInterpolateAcc(algorithmInterface);
}
void feedGyr(double ts, Eigen::Vector3d gyr_msg, SlamTester::AlgorithmInterface *algorithmInterface) {
    gyro_cache_.emplace_back(ts, gyr_msg.x(), gyr_msg.y(), gyr_msg.z());
    cur_input = gyr;
    constructImuInterpolateAcc(algorithmInterface);
}


int main(int argc, char *argv[]) {
    gflags :: SetUsageMessage ( "Usage : run run_example.sh " ) ;
    // gflags::ShowUsageWithFlags(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    FLAGS_logtostdout = true;
    FLAGS_colorlogtostdout = true;
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    if (FLAGS_algorithm.empty() || FLAGS_dataset.empty() || FLAGS_dataBag.empty() || FLAGS_camConfig.empty()) {
        LOG(ERROR) << "Please provide the command line args, use --help to get a clue.";
        exit(0);
    }

    /// Set up input Dataset.
    std::shared_ptr<SlamTester::InputInterface> input_inter;
    if (FLAGS_dataset == "tum_rs") {
        input_inter = std::make_shared<SlamTester::TumRsDataset>(FLAGS_camConfig,
                                                                 FLAGS_imuConfig, FLAGS_ciExtrinsic, FLAGS_dataBag, FLAGS_groundTruth, FLAGS_rs_cam);
    } else if (FLAGS_dataset == "euroc") {
        input_inter = std::make_shared<SlamTester::EurocDataset>(FLAGS_camConfig,
                                                                 FLAGS_imuConfig, FLAGS_ciExtrinsic, FLAGS_dataBag, FLAGS_groundTruth);
    } else {
        LOG(ERROR) << "Unknown dataset.";
        exit(0);
    }

    /// Set up algorithm
    std::unique_ptr<SlamTester::AlgorithmInterface> algorithm_inter;
    if (FLAGS_algorithm == "vins_mono")
        algorithm_inter = std::make_unique<VinsAlgorithm>(FLAGS_algoConfig);
    else if (FLAGS_algorithm == "example_undistort")
        algorithm_inter = std::make_unique<SlamTester::PangolinFakeAlgorithm>();
    else if (FLAGS_algorithm == "dso") {
        algorithm_inter = std::make_unique<DsoAlgorithm>();
    }
    else {
        LOG(ERROR) << "Unknown algorithm.";
        exit(0);
    }

    /// Set up outputs.
    std::shared_ptr<SlamTester::PangolinViewer> pango_viewer;
    if (FLAGS_resizeAndUndistort)
        pango_viewer = std::make_shared<SlamTester::PangolinViewer>(input_inter->orig_w,
         input_inter->orig_h, input_inter->inner_w, input_inter->inner_h,
         input_inter->gt_poses,input_inter->gt_times_s, false);
    else
        pango_viewer = std::make_shared<SlamTester::PangolinViewer>(input_inter->orig_w,
            input_inter->orig_h, input_inter->orig_w, input_inter->orig_h,
             input_inter->gt_poses,input_inter->gt_times_s, false);

    algorithm_inter->input_interfaces.push_back(input_inter);
    algorithm_inter->output_interfaces.push_back(pango_viewer);

    algorithm_inter->start();

    // Set up data threads.
    std::thread dataThread([&]() {
        pango_viewer->blockWaitForStart();

        ob_slam::rosbag::Bag play_bag;
        play_bag.open(input_inter->data_bag, static_cast<uint32_t>(ob_slam::rosbag::BagMode::Read));
        ob_slam::rosbag::View view(play_bag, ob_slam::rosbag::TopicQuery(input_inter->bag_topics),
                                   ob_slam::TIME_MIN, ob_slam::TIME_MAX);
        ob_slam::rosbag::View::iterator iter; uint i;
        bool imu_synced = false;
        
        double firstMsgTime = view.begin()->getTime().toSec();
        double initial_offset = 0;
        struct timeval tv_start;
        gettimeofday(&tv_start, nullptr);

        for (iter = view.begin(), i = 0; iter != view.end(); iter++, i++) {
            // LOG_FIRST_N(INFO, 50) << "ros topic: " << iter->getTopic();

            if (!pango_viewer->isRunning())
                break;

            struct timeval tv_now; gettimeofday(&tv_now, nullptr);
            double sSinceStart = (tv_now.tv_sec-tv_start.tv_sec + (tv_now.tv_usec-tv_start.tv_usec)/1e6) - initial_offset;
            double bag_time_since_start = (iter->getTime().toSec() - firstMsgTime) / pango_viewer->getPlayRate();

            if (bag_time_since_start == 0) {
                initial_offset = sSinceStart;
            }
            else if (sSinceStart < bag_time_since_start)
                usleep( ( bag_time_since_start - sSinceStart) * 1e6 );
            // Since we didn't launch different threads for different sensor streams like online running,
            // we choose not to skip imu messages to stay closer to online scenario.
            else if (input_inter->monoImg_topic == iter->getTopic() && FLAGS_skipCamMsgIfLag &&
                    sSinceStart - bag_time_since_start > 0.033/pango_viewer->getPlayRate()) {// Skip rgb msg if lagging more than 33ms.
                // LOG(WARNING) << "Skipped msg topic: " << iter->getTopic();
                LOG(WARNING) << "[SKIPPED ROS MSG]Skipped rgb msg at bag time: " << std::to_string(iter->getTime().toSec());
                continue;
            }


            if (input_inter->monoImg_topic == iter->getTopic()) {
                cv::Mat mat_out;
                double ts_sec;
                if (iter->isType<ob_slam::sensor_msgs::Image>()) {
                    ob_slam::sensor_msgs::Image::ConstPtr image_ptr = iter->instantiate<ob_slam::sensor_msgs::Image>();
                    CvBridgeSimple cvb;
                    mat_out = cvb.ConvertToCvMat(image_ptr);
                    ts_sec = image_ptr->header.stamp.toSec();
                } else if (iter->isType<ob_slam::sensor_msgs::CompressedImage>()) { //compressed image
                    ob_slam::sensor_msgs::CompressedImage::ConstPtr image_ptr = iter->instantiate<ob_slam::sensor_msgs::CompressedImage>();
                    mat_out = cv::imdecode(image_ptr->data, cv::IMREAD_GRAYSCALE);
                    ts_sec = image_ptr->header.stamp.toSec();
                } else {
                    LOG(ERROR) << "Unknown mono image type in ros bag!";
                    return;
                }
                if (FLAGS_showOrigCamStream) {
                    for (auto &oi: algorithm_inter->output_interfaces) {
                        oi->publishVideoImg(mat_out);
                    }
                }
                cv::Mat_<float> undistorted_mat;
                algorithm_inter->input_interfaces[0]->undistortImg(mat_out, undistorted_mat);
                algorithm_inter->feedMonoImg(ts_sec, undistorted_mat);
            }

            else if (input_inter->imu_topic == iter->getTopic()) {
                imu_synced = true;
                ob_slam::sensor_msgs::Imu::ConstPtr imu_ptr = iter->instantiate<ob_slam::sensor_msgs::Imu>();
                algorithm_inter->feedImu(imu_ptr->header.stamp.toSec(),
                                         Eigen::Vector3d(imu_ptr->linear_acceleration.x, imu_ptr->linear_acceleration.y,
                                                        imu_ptr->linear_acceleration.z),
                                         Eigen::Vector3d(imu_ptr->angular_velocity.x,
                                                        imu_ptr->angular_velocity.y, imu_ptr->angular_velocity.z));
            }
            else if (!imu_synced && input_inter->acc_topic == iter->getTopic()) {
                ob_slam::geometry_msgs::Vector3Stamped::ConstPtr acc_ptr =
                        iter->instantiate<ob_slam::geometry_msgs::Vector3Stamped>();
                feedAcc(acc_ptr->header.stamp.toSec(),
                                        Eigen::Vector3d(acc_ptr->vector.x, acc_ptr->vector.y, acc_ptr->vector.z), algorithm_inter.get());
            }
            else if (!imu_synced && input_inter->gyr_topic == iter->getTopic()) {
                ob_slam::geometry_msgs::Vector3Stamped::ConstPtr gyr_ptr =
                        iter->instantiate<ob_slam::geometry_msgs::Vector3Stamped>();
                feedGyr(gyr_ptr->header.stamp.toSec(),
                                        Eigen::Vector3d(gyr_ptr->vector.x, gyr_ptr->vector.y, gyr_ptr->vector.z), algorithm_inter.get());
            }
        }

        play_bag.close();
    });

    pango_viewer->run();// Make macOS happy.
    // std::thread pango_thread([&]{pango_viewer->run();});

    dataThread.join();
    LOG(INFO) << "dataThread joined.";

    algorithm_inter->stop();
    LOG(INFO) << "algorithm stopped.";

    return 0;
}
