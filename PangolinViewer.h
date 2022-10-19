/**
* This file is modified based on PangolinDSOViewer.h
*/


#pragma once

#include <pangolin/pangolin.h>
#include <thread>
#include <map>
#include <deque>
#include "MinimalImage.h"
#include "SlamInterface.h"

namespace SlamTester {

    class PangolinViewer : public OutputInterface {
    public:

        PangolinViewer(int video_w, int video_h, int algo_w, int algo_h, bool startRunThread = true);

        ~PangolinViewer() override;

        void run();
        void close();
        bool isRunning();
        void join();
        void reset();
        void blockWaitForStart();
        float getPlayRate();

        // ==================== Slam results interface ======================
        void publishCamPose(Eigen::Matrix4d &cam_pose) override;
        void publishImuPose(Eigen::Matrix4d &imu_pose) override;


        void publishVideoImg(cv::Mat video_img) override;
        void publishProcessImg(cv::Mat process_img) override;
        void publishImuMsg(Eigen::Vector3d acc, Eigen::Vector3d gyr) override;

    private:

        bool needReset;

        void reset_internal();

        // void drawConstraints();
        void drawPose(float lineWidth, float *color, float sizeFactor, Eigen::Matrix4f pose);

        std::thread runThread;
        bool ownThread;
        bool running;
        int video_w, video_h, algo_w, algo_h;

        // images rendering
        std::mutex openImagesMutex;
        std::unique_ptr<MinimalImageB3> videoImg;
        std::unique_ptr<MinimalImageB3> processImg;
        bool videoImgChanged, processImgChanged;


        // 3D model rendering
        std::mutex model3DMutex;
//	KeyFrameDisplay* currentCam;
        Eigen::Matrix4f camToWorld, imuToWorld;
//	std::vector<KeyFrameDisplay*> keyframes;
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> allFramePoses;
//	std::map<int, KeyFrameDisplay*> keyframesByKFID;
//	std::vector<GraphConnection,Eigen::aligned_allocator<GraphConnection>> connections;


        // render settings
        bool setting_render_showKFCameras;
        bool setting_render_showCurrentCamera;
        bool setting_render_showCurrentImu;
        bool setting_render_showTrajectory;
        bool setting_render_showActiveConstraints;
        bool setting_render_showAllConstraints;
        bool setting_render_display3D;
        bool setting_render_displayVideo;
        bool setting_render_displayProcess;
        bool setting_render_fullResetRequested;
        bool control_started = false;
        float setting_playback_rate;

        // timings
        struct timeval last_camPose_t;
        struct timeval last_imuPose_t;
        struct timeval last_rgbMsg_t;
        struct timeval last_imuMsg_t;
        struct timeval last_processImg_t;

        std::deque<float> lastNimuPoseMs;
        std::deque<float> lastNcamPoseMs;
        std::deque<float> lastNimuMsgMs;
        std::deque<float> lastNrgbMsgMs;
        std::deque<float> lastNprocessImgMs;

        // colors
        float red[3]{1.0, 0, 0};
        float green[3]{0, 1.0, 0};
        float blue[3]{0, 0, 1.0};
    };


}




