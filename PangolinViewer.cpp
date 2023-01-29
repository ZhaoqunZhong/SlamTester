/**
* This file is modified based on PangolinViewer.cpp
*/


#include <sys/time.h>
#include "PangolinViewer.h"
#include "Eigen/Core"
#include "glog/logging.h"
#include "alignment/AlignTrajectory.h"

namespace SlamTester {

    PangolinViewer::PangolinViewer(int video_w, int video_h, int algo_w, int algo_h,
                                   std::vector<Eigen::Matrix<double, 7, 1>> &gt, std::vector<double> &gt_time,
                                   bool startRunThread) {
        this->video_w = video_w;
        this->video_h = video_h;
        this->algo_w = algo_w;
        this->algo_h = algo_h;
        gt_poses = gt;
        gt_times_s = gt_time;
        running = true;
        ownThread = startRunThread;

        {
            std::unique_lock<std::mutex> lk(openImagesMutex);
            videoImg = std::make_unique<MinimalImageB3>(video_w, video_h);
            processImg = std::make_unique<MinimalImageB3>(algo_w, algo_h);
            videoImgChanged = processImgChanged = true;

            videoImg->setBlack();
            processImg->setBlack();
//        videoImg->setConst(Vec3b(255,255,255));
//        processImg->setConst(Vec3b(255,255,255));

        }


        {
//		currentCam = new KeyFrameDisplay();
            imuToWorld = camToWorld = Eigen::Matrix4d::Identity();
        }

        needReset = false;


        if (ownThread)
            runThread = std::thread(&PangolinViewer::run, this);

    }


    PangolinViewer::~PangolinViewer() {
        close();
        if (ownThread)
            runThread.join();
    }


    void PangolinViewer::run() {
        printf("START PANGOLIN!\n");

        pangolin::CreateWindowAndBind("Main", 1000, 800);
        const int UI_WIDTH = 300;

        glEnable(GL_DEPTH_TEST);

        // 3D visualization
        pangolin::OpenGlRenderState Visualization3D_camera(
                pangolin::ProjectionMatrix(video_w, video_h, 400, 400, video_w / 2, video_h / 2, 0.1, 1000),
                pangolin::ModelViewLookAt(-0, -5, -10, 0, 0, 0, pangolin::AxisNegY)
        );

        pangolin::View &Visualization3D_display = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -video_w / (float) video_h)
                .SetHandler(new pangolin::Handler3D(Visualization3D_camera));


        // 2 images
        pangolin::View &d_video = pangolin::Display("imgVideo")
                .SetAspect(video_w / (float) video_h);

        pangolin::View &d_process = pangolin::Display("imgProcess")
                .SetAspect(algo_w / (float) algo_h);


        pangolin::GlTexture texProcess(algo_w, algo_h, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
        pangolin::GlTexture texVideo(video_w, video_h, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);


        pangolin::CreateDisplay()
                .SetBounds(0.0, 0.3, pangolin::Attach::Pix(UI_WIDTH), 1.0)
                // .SetLock(pangolin::LockRight, pangolin::LockBottom)
                .SetLayout(pangolin::LayoutEqual)
                .AddDisplay(d_video)
                .AddDisplay(d_process);


        // parameter reconfigure gui
        pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
        pangolin::Var<bool> startButton("ui.Start", false, false);
//	pangolin::Var<int> settings_pointCloudMode("ui.PC_mode",1,1,4,false);
        pangolin::Var<float> settings_playRate("ui.PlayBackRate",1,0.25,4,false);
        pangolin::Var<bool> settings_showKFCameras("ui.KFCam", false, true);
        pangolin::Var<bool> settings_showCurrentCamera("ui.CurrCam", false, true);
        pangolin::Var<bool> settings_showCurrentImu("ui.CurrImu", true, true);
        pangolin::Var<bool> settings_showCamTrajectory("ui.camPoses", false, true);
        pangolin::Var<bool> settings_showImuTrajectory("ui.imuPoses", true, true);
        pangolin::Var<bool> settings_showGroundTruth("ui.GroundTruth", true, true);
        pangolin::Var<bool> settings_showActiveConstraints("ui.ActiveConst", true, true);
        pangolin::Var<bool> settings_showAllConstraints("ui.AllConst", false, true);
        pangolin::Var<bool> settings_show3D("ui.show3D", true, true);
        pangolin::Var<bool> settings_showLiveProcess("ui.showProcess", true, true);
        pangolin::Var<bool> settings_showLiveVideo("ui.showVideo", true, true);
        pangolin::Var<bool> resetButton("ui.Reset", false, false);
        pangolin::Var<double> display_imuFps("ui.ImuPoseFps", 0, 0, 0, false);
        pangolin::Var<double> display_camFps("ui.CamPoseFps", 0, 0, 0, false);
        pangolin::Var<double> display_rgbFps("ui.rgbMsgFps", 0, 0, 0, false);
        pangolin::Var<double> display_imuMsgFps("ui.imuMsgFps", 0, 0, 0, false);
        pangolin::Var<double> display_processImgFps("ui.processImgFps", 0, 0, 0, false);

        struct timeval last_align_time;
        gettimeofday(&last_align_time, nullptr);
        // Default hooks for exiting (Esc) and fullscreen (tab).
        while (!pangolin::ShouldQuit() && running) {
            // Clear entire screen
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            if (setting_render_display3D) {
                // Activate efficiently by object
                Visualization3D_display.Activate(Visualization3D_camera);
                // lock
                std::unique_lock<std::mutex> lk3d(model3DMutex);

                //pangolin::glDrawColouredCube();
                if (setting_render_showCurrentImu) {
                    drawPose(2, green, 0.4, imuToWorld.cast<float>());
                }
                if (setting_render_showCurrentCamera) {
                    drawPose(2, blue, 0.4, camToWorld.cast<float>());
                }
                if (setting_render_showImuTrajectory) {
                    glColor3f(green[0], green[1], green[2]);
                    glLineWidth(3);
                    glBegin(GL_LINE_STRIP);
                    for(unsigned int i=0; i < imuPoses.size(); i++)
                    {
                        Eigen::Vector4d imu_tran(0,0,0,1);
                        imu_tran.block<3,1>(0,0) = imuPoses[i].block<3,1>(0,0);
                        auto aligned_tran = (gtToTraj * imu_tran).cast<float>();
                        glVertex3f(aligned_tran.x(), aligned_tran.y(), aligned_tran.z());
                    }
                    glEnd();
                }
                if (setting_render_showCamTrajectory) {
                    glColor3f(blue[0], blue[1], blue[2]);
                    glLineWidth(3);
                    glBegin(GL_LINE_STRIP);
                    for(unsigned int i=0; i < camPoses.size(); i++)
                    {
                        auto cam_tran = camPoses[i].cast<float>().block<3,1>(0,0);
                        glVertex3f(cam_tran.x(), cam_tran.y(), cam_tran.z());
                    }
                    glEnd();
                }
                if (setting_render_showGroundTruth) {
                    glColor3f(red[0], red[1], red[2]);
                    glLineWidth(3);
                    glBegin(GL_LINE_STRIP);
                    for (unsigned int i = 0; i < gt_poses.size(); ++i) {
                        auto tran = gt_poses[i].block<3,1>(0,0);
                        glVertex3f(tran.x(), tran.y(), tran.z());
                    }
                    glEnd();
                }


                lk3d.unlock();
            }


            openImagesMutex.lock();
            if (videoImgChanged) texVideo.Upload(videoImg->data, GL_BGR, GL_UNSIGNED_BYTE);
            if (processImgChanged) texProcess.Upload(processImg->data, GL_BGR, GL_UNSIGNED_BYTE);
            videoImgChanged = processImgChanged = false;
            openImagesMutex.unlock();


            // update fps counters
            model3DMutex.lock();
            {
                // model3DMutex.lock();
                float sd = 0;
                for (float d: lastNcamPoseMs) sd += d;
                display_camFps = lastNcamPoseMs.size() * 1000.0f / sd;
                // model3DMutex.unlock();
            }
            {
                // model3DMutex.lock();
                float sd = 0;
                for (float d: lastNimuPoseMs) sd += d;
                display_imuFps = lastNimuPoseMs.size() * 1000.0f / sd;
                // model3DMutex.unlock();
            }
            {
                // model3DMutex.lock();
                float sd = 0;
                for (float d: lastNrgbMsgMs) sd += d;
                display_rgbFps = lastNrgbMsgMs.size() * 1000.0f / sd;
                // model3DMutex.unlock();
            }
            {
                float sd = 0;
                for (float d: lastNimuMsgMs) sd += d;
                display_imuMsgFps = lastNimuMsgMs.size() * 1000.0f / sd;
            }
            {
                float sd = 0;
                for (float d: lastNprocessImgMs) sd += d;
                display_processImgFps = lastNprocessImgMs.size() * 1000.0f / sd;
            }
            model3DMutex.unlock();


            if (setting_render_displayVideo) {
                d_video.Activate();
                glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
                texVideo.RenderToViewportFlipY();
            }
            if (setting_render_displayProcess) {
                d_process.Activate();
                glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
                texProcess.RenderToViewportFlipY();
            }



            // update parameters
            setting_render_showActiveConstraints = settings_showActiveConstraints.Get();
            setting_render_showAllConstraints = settings_showAllConstraints.Get();
            setting_render_showCurrentCamera = settings_showCurrentCamera.Get();
            setting_render_showCurrentImu = settings_showCurrentImu.Get();
            setting_render_showKFCameras = settings_showKFCameras.Get();
            setting_render_showImuTrajectory = settings_showImuTrajectory.Get();
            setting_render_showCamTrajectory = settings_showCamTrajectory.Get();
            setting_render_showGroundTruth = settings_showGroundTruth.Get();
            setting_render_display3D = settings_show3D.Get();
            setting_render_displayProcess = settings_showLiveProcess.Get();
            setting_render_displayVideo = settings_showLiveVideo.Get();
            control_started = startButton.Get();
            setting_playback_rate = settings_playRate.Get();


            if (resetButton.Get()) {
                printf("RESET!\n");
                resetButton.Reset();
                setting_render_fullResetRequested = true;
            }

            // Swap frames and Process Events
            pangolin::FinishFrame();

            /// Align trajectory with ground truth.
            struct timeval time_now;
            gettimeofday(&time_now, nullptr);
            if ((time_now.tv_sec - last_align_time.tv_sec) * 1.0f + (time_now.tv_usec - last_align_time.tv_usec) / 1000000.f > 2) {
                last_align_time = time_now;
                // std::thread align_thread(&PangolinViewer::alignTrajToGt, this, 0, 0.02, imuPoses, imu_times_s, gt_poses, gt_times_s, gtToTraj);
                std::thread align_thread(&PangolinViewer::alignTrajToGt, this, 0, 0.02, std::ref(imuPoses), std::ref(imu_times_s),
                                         std::ref(gt_poses), std::ref(gt_times_s), std::ref(gtToTraj));
                align_thread.detach();
            }
/*            // Debug traj alignment
            struct timeval time_now;
            gettimeofday(&time_now, nullptr);
            if ((time_now.tv_sec - last_align_time.tv_sec) * 1.0f + (time_now.tv_usec - last_align_time.tv_usec) / 1000000.f > 2) {
                alignTrajToGt(0, 0.02, imuPoses, imu_times_s, gt_poses, gt_times_s, gtToTraj);
                last_align_time = time_now;
            }*/

            if (needReset) reset_internal();
        }


        printf("QUIT Pangolin thread!\n");
        // printf("I'll just kill the whole process.\nSo Long, and Thanks for All the Fish!\n");

        // exit(1);
        close();
    }


    void PangolinViewer::close() {
        running = false;
    }

    bool PangolinViewer::isRunning() {
        return running;
    }

    void PangolinViewer::join() {
        runThread.join();
        printf("JOINED Pangolin thread!\n");
    }

    void PangolinViewer::reset() {
        needReset = true;
    }

    void PangolinViewer::reset_internal() {
        model3DMutex.lock();
        // for(size_t i=0; i<keyframes.size();i++) delete keyframes[i];
        // keyframes.clear();
        imuPoses.clear();
        camPoses.clear();
        // keyframesByKFID.clear();
        // connections.clear();
        model3DMutex.unlock();


        openImagesMutex.lock();
        videoImg->setBlack();
        processImg->setBlack();
        videoImgChanged = processImgChanged = true;
        openImagesMutex.unlock();

        needReset = false;
    }

    void PangolinViewer::blockWaitForStart() {
        while (!control_started && running)
            usleep(5 * 1e3);
    }

    float PangolinViewer::getPlayRate() {
        return setting_playback_rate;
    }

    void PangolinViewer::publishCamPose(Eigen::Matrix4d &cam_pose, double ts) {
        if (!setting_render_display3D) return;
        if (!setting_render_showCurrentCamera) return;

        std::unique_lock<std::mutex> lk(model3DMutex);
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        lastNcamPoseMs.push_back(
                ((time_now.tv_sec - last_camPose_t.tv_sec) * 1000.0f + (time_now.tv_usec - last_camPose_t.tv_usec) / 1000.0f));
        if (lastNcamPoseMs.size() > 10) lastNcamPoseMs.pop_front();
        last_camPose_t = time_now;

        camToWorld = cam_pose;
        Eigen::Matrix<double,7,1> cam_pose7;
        cam_pose7.block<3,1>(0,0) = camToWorld.block<3,1>(0,3);
        cam_pose7.block<4,1>(3,0) = AlignUtils::rot_2_quat(camToWorld.block<3,3>(0,0));
        camPoses.push_back(cam_pose7);
        cam_times_s.push_back(ts);
    }

    void PangolinViewer::publishImuPose(Eigen::Matrix4d &imu_pose, double ts) {
        if (!setting_render_display3D) return;
        if (!setting_render_showCurrentImu) return;

        std::unique_lock<std::mutex> lk(model3DMutex);
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        lastNimuPoseMs.push_back(
                ((time_now.tv_sec - last_imuPose_t.tv_sec) * 1000.0f + (time_now.tv_usec - last_imuPose_t.tv_usec) / 1000.0f));
        if (lastNimuPoseMs.size() > 30) lastNimuPoseMs.pop_front();
        last_imuPose_t = time_now;

        imuToWorld = imu_pose;
        Eigen::Matrix<double,7,1> imu_pose7;
        imu_pose7.block<3,1>(0,0) = imuToWorld.block<3,1>(0,3);
        imu_pose7.block<4,1>(3,0) = AlignUtils::rot_2_quat(imuToWorld.block<3,3>(0,0));
        imuPoses.push_back(imu_pose7);
        imu_times_s.push_back(ts);
    }

    void PangolinViewer::publishVideoImg(cv::Mat video_img) {
        if (!setting_render_displayVideo) return;
        
        if (video_img.depth() == CV_8U) {}
        else if (video_img.depth() == CV_16U)
            video_img.convertTo(video_img, CV_8U, 1.0/256);
        else {
            LOG(ERROR) << "publishVideoImg: Doesn't support this data type."; 
            return;
        }
        
        cv::Mat rgb;
        if (video_img.channels() == 1) {
            cv::cvtColor(video_img, rgb, cv::COLOR_GRAY2RGB);
        } else if (video_img.channels() == 3) {
            rgb = video_img;
        } else {
            LOG(ERROR) << "publishVideoImg: video_img incorrect num of channels!";
            return;
        }

        std::unique_lock<std::mutex> lk(openImagesMutex);

        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        lastNrgbMsgMs.push_back(
                ((time_now.tv_sec - last_rgbMsg_t.tv_sec) * 1000.0f + (time_now.tv_usec - last_rgbMsg_t.tv_usec) / 1000.0f));
        if (lastNrgbMsgMs.size() > 10) lastNrgbMsgMs.pop_front();
        last_rgbMsg_t = time_now;

        memcpy(videoImg->data, rgb.data, video_w * video_h *3);
        videoImgChanged = true;
        
/*        struct timeval time0;
        gettimeofday(&time0, NULL);

        if (rgb.depth() == CV_16U) {
            typedef cv::Point3_<uint16_t> Pixel;
            rgb.forEach<Pixel>([this](Pixel &p, const int *position) -> void {
                int idx = position[1] + position[0] * this->video_w;
                videoImg->data[idx][0] = p.x / 256;
                videoImg->data[idx][1] = p.y / 256;
                videoImg->data[idx][2] = p.z / 256;
            });
        } else if (rgb.depth() == CV_8U) {
            typedef cv::Point3_<uint8_t> Pixel;
            rgb.forEach<Pixel>([this](Pixel &p, const int *position) -> void {
                int idx = position[1] + position[0] * this->video_w;
                videoImg->data[idx][0] = p.x;
                videoImg->data[idx][1] = p.y;
                videoImg->data[idx][2] = p.z;
            });
        } else {
            LOG(ERROR) << "publishVideoImg: rgb img incorrect data type!";
            return;
        }

        struct timeval time1;
        gettimeofday(&time1, NULL);
        LOG_FIRST_N(INFO, 20) << "forEach function costs " <<
            (time1.tv_sec - time0.tv_sec) * 1000.0f + (time1.tv_usec - time0.tv_usec) / 1000.0f;*/

        // Equally effective method for CV_16U.
/*        cv::Mat rgb888;
        rgb.convertTo(rgb888, CV_8U, 1.0/256);
        memcpy(videoImg->data, rgb888.data, video_w*video_h*3);

        struct timeval time2;
        gettimeofday(&time2, NULL);
        LOG_FIRST_N(INFO, 20) << "convertTo and memcpy function costs " <<
            (time2.tv_sec - time1.tv_sec) * 1000.0f + (time2.tv_usec - time1.tv_usec) / 1000.0f;*/

    }

    void PangolinViewer::publishProcessImg(cv::Mat process_img) {
        if (!setting_render_displayProcess) return;

        if (process_img.depth() == CV_8U) {}
        else if (process_img.depth() == CV_16U)
            process_img.convertTo(process_img, CV_8U, 1.0/256);
/*        else if (process_img.depth() == CV_16F)
            process_img.convertTo(process_img, CV_8U, 1);*/
        else {
            LOG(ERROR) << "publishProcessImg: Doesn't support this data type.";
            return;
        }

        cv::Mat rgb;
        if (process_img.channels() == 1) {
            cv::cvtColor(process_img, rgb, cv::COLOR_GRAY2RGB);
        } else if (process_img.channels() == 3) {
            rgb = process_img;
        } else {
            LOG(ERROR) << "publishProcessImg: process_img incorrect num of channels!";
            return;
        }

        std::unique_lock<std::mutex> lk(openImagesMutex);

        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        lastNprocessImgMs.push_back(
                ((time_now.tv_sec - last_processImg_t.tv_sec) * 1000.0f + (time_now.tv_usec - last_processImg_t.tv_usec) / 1000.0f));
        if (lastNprocessImgMs.size() > 10) lastNprocessImgMs.pop_front();
        last_processImg_t = time_now;

        memcpy(processImg->data, rgb.data, algo_w * algo_h *3);
        processImgChanged = true;
    }
    
    void PangolinViewer::publishImuMsg(Eigen::Vector3d acc, Eigen::Vector3d gyr) {
        std::unique_lock<std::mutex> lk(openImagesMutex);

        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        lastNimuMsgMs.push_back(
                ((time_now.tv_sec - last_imuMsg_t.tv_sec) * 1000.0f + (time_now.tv_usec - last_imuMsg_t.tv_usec) / 1000.0f));
        if (lastNimuMsgMs.size() > 30) lastNimuMsgMs.pop_front();
        last_imuMsg_t = time_now;
    }


    void PangolinViewer::drawPose(float lineWidth, float *color, float sizeFactor, Eigen::Matrix4f pose) {

        int width = 640, height = 480;
        float fx = 460, fy = 460, cx = width / 2.0, cy = height / 2.0;
        float sz = sizeFactor;

        glPushMatrix();
//    Sophus::Matrix4f m = camToWorld.matrix().cast<float>();
        glMultMatrixf((GLfloat *) pose.data());

        if (color == 0) {
            glColor3f(0, 0, 1);
        } else
            glColor3f(color[0], color[1], color[2]);

        glLineWidth(lineWidth);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glEnd();
        glPopMatrix();
    }

    void PangolinViewer::alignTrajToGt(double t_offset_traj, double max_t_diff, std::vector<Eigen::Matrix<double, 7, 1>> &traj,
                                       std::vector<double> &traj_times, std::vector<Eigen::Matrix<double, 7, 1>> &gt,
                                       std::vector<double> &gt_times, Eigen::Matrix4d &transform) {

        model3DMutex.lock();
        auto traj_copy = traj;
        auto traj_time_copy = traj_times;
        auto gt_pose_copy = gt;
        auto gt_time_copy = gt_times;
        model3DMutex.unlock();

        AlignUtils::perform_association(t_offset_traj, max_t_diff,
                                        traj_time_copy, gt_time_copy, traj_copy, gt_pose_copy);

        if (traj_copy.size() < 3)
            return;

        Eigen::Matrix3d R_gtToTraj;
        Eigen::Vector3d t_gtToTraj;
        double s_trajToGt;

        AlignTrajectory::align_trajectory(traj_copy, gt_pose_copy, R_gtToTraj, t_gtToTraj, s_trajToGt, "sim3");

        model3DMutex.lock();
        transform.setIdentity();
        transform.block<3,3>(0,0) = R_gtToTraj * s_trajToGt;
        transform.block<3,1>(0,3) = t_gtToTraj;
        model3DMutex.unlock();
    }


}

