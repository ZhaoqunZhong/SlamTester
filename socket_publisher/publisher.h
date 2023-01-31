#ifndef SOCKET_PUBLISHER_PUBLISHER_H
#define SOCKET_PUBLISHER_PUBLISHER_H

#include "data_serializer.h"
#include "socket_client.h"

#include <mutex>
#include <memory>

#include "../SlamInterface.h"
#include "yaml-cpp/yaml.h"

// namespace stella_vslam {

// class config;
// class system;

// namespace publish {
// class frame_publisher;
// class map_publisher;
// } // namespace publish

// } // namespace stella_vslam

namespace socket_publisher {

    class publisher: public SlamTester::OutputInterface {
    public:
    publisher(const YAML::Node& yaml_node/*,
              const std::shared_ptr<stella_vslam::system>& system,
              const std::shared_ptr<stella_vslam::publish::frame_publisher>& frame_publisher,
              const std::shared_ptr<stella_vslam::publish::map_publisher>& map_publisher*/);

        void run();

    /* thread controls */
        void request_pause();
        bool is_paused();
        void resume();
        void request_terminate();
        bool is_terminated();

            // ==================== Slam results interface ======================
        void publishCamPose(Eigen::Matrix4d &cam_pose, double ts) override;
        void publishImuPose(Eigen::Matrix4d &imu_pose, double ts) override;
        // void publishVideoImg(cv::Mat video_img) override;
        void publishProcessImg(cv::Mat process_img) override;
        // void publishImuMsg(Eigen::Vector3d acc, Eigen::Vector3d gyr) override;

    private:
    // const std::shared_ptr<stella_vslam::system> system_;
        const unsigned int emitting_interval_;
        const unsigned int image_quality_;

        std::unique_ptr<socket_client> client_;
        std::unique_ptr<data_serializer> data_serializer_;

        void callback(const std::string& message);

    /* thread controls */
        bool pause_if_requested();

        bool terminate_is_requested();
        void terminate();

        std::mutex mtx_terminate_;
        bool terminate_is_requested_ = false;
        bool is_terminated_ = true;

        std::mutex mtx_pause_;
        bool pause_is_requested_ = false;
        bool is_paused_ = true;

        std::mutex mtx_processed_img_;
        bool processed_img_changed_ = false;
        cv::Mat processed_img_;

        std::mutex mtx_latest_cam_pose_;
        Eigen::Matrix4d latest_cam_pose_;
        std::mutex mtx_latest_imu_pose_;
        Eigen::Matrix4d latest_imu_pose_;
        std::string pose_source_;
    };

} // namespace socket_publisher

#endif // SOCKET_PUBLISHER_PUBLISHER_H
