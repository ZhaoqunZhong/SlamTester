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


// Choose dataset & algorithm.
#include "TumRs_Pangolin_Example.h"
std::unique_ptr<SlamTester::AlgorithmInterface> algorithm_inter = std::make_unique<SlamTester::PangolinFakeAlgorithm>();


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
void constructImuInterpolateAcc() {
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
                    algorithm_inter->feedImu(imu.ts, Eigen::Vector3d{imu.acc_part.ax, imu.acc_part.ay, imu.acc_part.az},
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
                            algorithm_inter->feedImu(imu.ts, Eigen::Vector3d{imu.acc_part.ax, imu.acc_part.ay, imu.acc_part.az},
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
                        algorithm_inter->feedImu(imu.ts, Eigen::Vector3d{imu.acc_part.ax, imu.acc_part.ay, imu.acc_part.az},
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
                    algorithm_inter->feedImu(imu.ts, Eigen::Vector3d{imu.acc_part.ax, imu.acc_part.ay, imu.acc_part.az},
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
                    algorithm_inter->feedImu(imu.ts, Eigen::Vector3d{imu.acc_part.ax, imu.acc_part.ay, imu.acc_part.az},
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
                    algorithm_inter->feedImu(imu.ts, Eigen::Vector3d{imu.acc_part.ax, imu.acc_part.ay, imu.acc_part.az},
                            Eigen::Vector3d{imu.gyro_part.rx, imu.gyro_part.ry, imu.gyro_part.rz});
                    gyro_cache_.erase(gyro_cache_.begin());
                    acc_cache_.erase(acc_cache_.begin(), acc_cache_.begin() + i-1);
                }
            }
            break;
    }
}
void feedAcc(double ts, Eigen::Vector3d acc_msg) {
    acc_cache_.emplace_back(ts, acc_msg.x(), acc_msg.y(), acc_msg.z());
    cur_input = acc;
    constructImuInterpolateAcc();
}
void feedGyr(double ts, Eigen::Vector3d gyr_msg) {
    gyro_cache_.emplace_back(ts, gyr_msg.x(), gyr_msg.y(), gyr_msg.z());
    cur_input = gyr;
    constructImuInterpolateAcc();
}


int main(int argc, char **argv) {
    FLAGS_logtostdout = true;
    FLAGS_colorlogtostdout = true;
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    // Set up inputs.
    std::string ros_bag = "/Users/zhongzhaoqun/Downloads/dataset-seq1.bag";
    std::string cam_config = "/Users/zhongzhaoqun/Downloads/dataset-seq1/dso/cam1/camera copy.txt";
    auto tumRS_input_pangolin = std::make_shared<SlamTester::TumRsPangolinInput>(cam_config, ros_bag);
    algorithm_inter->input_interfaces.push_back(tumRS_input_pangolin);

    // Set up outputs.
    auto pango_viewer = std::make_shared<SlamTester::PangolinViewer>(tumRS_input_pangolin->orig_w,
         tumRS_input_pangolin->orig_h, tumRS_input_pangolin->inner_w, tumRS_input_pangolin->inner_h, false);
    algorithm_inter->output_interfaces.push_back(pango_viewer);

    algorithm_inter->start();

    std::thread tumRSdataThread([&]() {
        pango_viewer->blockWaitForStart();

        ob_slam::rosbag::Bag play_bag;
        play_bag.open(tumRS_input_pangolin->data_bag, static_cast<uint32_t>(ob_slam::rosbag::BagMode::Read));
        ob_slam::rosbag::View view(play_bag, ob_slam::rosbag::TopicQuery(tumRS_input_pangolin->bag_topics),
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
            else if (tumRS_input_pangolin->monoImg_topic == iter->getTopic() &&
                    sSinceStart - bag_time_since_start > 0.033/pango_viewer->getPlayRate()) {// Skip rgb msg if lagging more than 33ms.
                // LOG(WARNING) << "Skipped msg topic: " << iter->getTopic();
                LOG(WARNING) << "Skipped rgb msg at bag time: " << std::to_string(iter->getTime().toSec());
                continue;
            }

            if (tumRS_input_pangolin->monoImg_topic == iter->getTopic()) {
                if (iter->isType<ob_slam::sensor_msgs::Image>()) {
                    ob_slam::sensor_msgs::Image::ConstPtr image_ptr = iter->instantiate<ob_slam::sensor_msgs::Image>();
                    CvBridgeSimple cvb;
                    cv::Mat mat = cvb.ConvertToCvMat(image_ptr);
                    algorithm_inter->feedMonoImg(image_ptr->header.stamp.toSec(), mat);
                } else if (iter->isType<ob_slam::sensor_msgs::CompressedImage>()) { //compressed image
                    ob_slam::sensor_msgs::CompressedImage::ConstPtr image_ptr = iter->instantiate<ob_slam::sensor_msgs::CompressedImage>();
                    cv::Mat mat = cv::imdecode(image_ptr->data, cv::IMREAD_GRAYSCALE);
                    algorithm_inter->feedMonoImg(image_ptr->header.stamp.toSec(), mat);
                } else {
                    LOG(ERROR) << "Unknown mono image type in ros bag!";
                    return;
                }
            }

            else if (tumRS_input_pangolin->imu_topic == iter->getTopic()) {
                imu_synced = true;
                ob_slam::sensor_msgs::Imu::ConstPtr imu_ptr = iter->instantiate<ob_slam::sensor_msgs::Imu>();
                algorithm_inter->feedImu(imu_ptr->header.stamp.toSec(),
                                        Eigen::Vector3d(imu_ptr->linear_acceleration.x, imu_ptr->linear_acceleration.y,
                                                        imu_ptr->linear_acceleration.z),
                                        Eigen::Vector3d(imu_ptr->angular_velocity.x,
                                                        imu_ptr->angular_velocity.y, imu_ptr->angular_velocity.z));
            }
            else if (!imu_synced && tumRS_input_pangolin->acc_topic == iter->getTopic()) {
                ob_slam::geometry_msgs::Vector3Stamped::ConstPtr acc_ptr =
                        iter->instantiate<ob_slam::geometry_msgs::Vector3Stamped>();
                feedAcc(acc_ptr->header.stamp.toSec(),
                                        Eigen::Vector3d(acc_ptr->vector.x, acc_ptr->vector.y, acc_ptr->vector.z));
            }
            else if (!imu_synced && tumRS_input_pangolin->gyr_topic == iter->getTopic()) {
                ob_slam::geometry_msgs::Vector3Stamped::ConstPtr gyr_ptr =
                        iter->instantiate<ob_slam::geometry_msgs::Vector3Stamped>();
                feedGyr(gyr_ptr->header.stamp.toSec(),
                                        Eigen::Vector3d(gyr_ptr->vector.x, gyr_ptr->vector.y, gyr_ptr->vector.z));
            }
        }

        play_bag.close();
    });

    pango_viewer->run();// Make macOS happy.

    tumRSdataThread.join();

    algorithm_inter->stop();

    return 0;
}
