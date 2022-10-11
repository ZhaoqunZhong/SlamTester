/**
* This file is modified based on PangolinViewer.cpp
*/



#include <sys/time.h>
#include "PangolinViewer.h"
#include "Eigen/Core"
#include "glog/logging.h"

namespace SlamTester {

    PangolinViewer::PangolinViewer(int w, int h, bool startRunThread) {
        this->w = w;
        this->h = h;
        running = true;
        ownThread = startRunThread;

        {
            std::unique_lock<std::mutex> lk(openImagesMutex);
            videoImg = std::make_unique<MinimalImageB3>(w, h);
            processImg = std::make_unique<MinimalImageB3>(w, h);
            videoImgChanged = processImgChanged = true;

            videoImg->setBlack();
            processImg->setBlack();
//        videoImg->setConst(Vec3b(255,255,255));
//        processImg->setConst(Vec3b(255,255,255));

        }


        {
//		currentCam = new KeyFrameDisplay();
            imuToWorld = camToWorld = Eigen::Matrix4f::Identity();
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

        pangolin::CreateWindowAndBind("Main", w, h);
        const int UI_WIDTH = 200;

        glEnable(GL_DEPTH_TEST);

        // 3D visualization
        pangolin::OpenGlRenderState Visualization3D_camera(
                pangolin::ProjectionMatrix(w, h, 400, 400, w / 2, h / 2, 0.1, 1000),
                pangolin::ModelViewLookAt(-0, -5, -10, 0, 0, 0, pangolin::AxisNegY)
        );

        pangolin::View &Visualization3D_display = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -w / (float) h)
                .SetHandler(new pangolin::Handler3D(Visualization3D_camera));


        // 2 images
        pangolin::View &d_process = pangolin::Display("imgProcess")
                .SetAspect(w / (float) h);

        pangolin::View &d_video = pangolin::Display("imgVideo")
                .SetAspect(w / (float) h);


        pangolin::GlTexture texProcess(w, h, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
        pangolin::GlTexture texVideo(w, h, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);


        pangolin::CreateDisplay()
                .SetBounds(0.0, 0.3, pangolin::Attach::Pix(UI_WIDTH), 1.0)
                .SetLayout(pangolin::LayoutEqual)
                .AddDisplay(d_process)
                .AddDisplay(d_video);

        // parameter reconfigure gui
        pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

//	pangolin::Var<int> settings_pointCloudMode("ui.PC_mode",1,1,4,false);

        pangolin::Var<bool> settings_showKFCameras("ui.KFCam", false, true);
        pangolin::Var<bool> settings_showCurrentCamera("ui.CurrCam", true, true);
        pangolin::Var<bool> settings_showCurrentImu("ui.CurrImu", false, true);
        pangolin::Var<bool> settings_showTrajectory("ui.Trajectory", true, true);
        pangolin::Var<bool> settings_showFullTrajectory("ui.FullTrajectory", false, true);
        pangolin::Var<bool> settings_showActiveConstraints("ui.ActiveConst", true, true);
        pangolin::Var<bool> settings_showAllConstraints("ui.AllConst", false, true);


        pangolin::Var<bool> settings_show3D("ui.show3D", true, true);
        pangolin::Var<bool> settings_showLiveProcess("ui.showProcess", true, true);
        pangolin::Var<bool> settings_showLiveVideo("ui.showVideo", true, true);

        pangolin::Var<bool> settings_resetButton("ui.Reset", false, false);


        pangolin::Var<double> settings_imuFps("ui.ImuPoseFps", 0, 0, 0, false);
        pangolin::Var<double> settings_camFps("ui.CamPoseFps", 0, 0, 0, false);
        pangolin::Var<double> settings_rgbFps("ui.rgbMsgFps", 0, 0, 0, false);
        pangolin::Var<double> settings_imuMsgFps("ui.imuMsgFps", 0, 0, 0, false);
        pangolin::Var<double> settings_processImgFps("ui.processImgFps", 0, 0, 0, false);

        // Default hooks for exiting (Esc) and fullscreen (tab).
        while (!pangolin::ShouldQuit() && running) {
            // Clear entire screen
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            if (setting_render_display3D) {
                // Activate efficiently by object
                Visualization3D_display.Activate(Visualization3D_camera);
                std::unique_lock<std::mutex> lk3d(model3DMutex);
                //pangolin::glDrawColouredCube();

                drawImu(2, nullptr, 0.1);
                drawCam(2, nullptr, 0.2);

//			drawConstraints();
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
                settings_camFps = lastNcamPoseMs.size() * 1000.0f / sd;
                // model3DMutex.unlock();
            }
            {
                // model3DMutex.lock();
                float sd = 0;
                for (float d: lastNimuPoseMs) sd += d;
                settings_imuFps = lastNimuPoseMs.size() * 1000.0f / sd;
                // model3DMutex.unlock();
            }
            {
                // model3DMutex.lock();
                float sd = 0;
                for (float d: lastNrgbMsgMs) sd += d;
                settings_rgbFps = lastNrgbMsgMs.size() * 1000.0f / sd;
                // model3DMutex.unlock();
            }
            {
                float sd = 0;
                for (float d: lastNimuMsgMs) sd += d;
                settings_imuMsgFps = lastNimuMsgMs.size() * 1000.0f / sd;
            }
            {
                float sd = 0;
                for (float d: lastNprocessImgMs) sd += d;
                settings_processImgFps = lastNprocessImgMs.size() * 1000.0f / sd;
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
            setting_render_showTrajectory = settings_showTrajectory.Get();
            setting_render_showFullTrajectory = settings_showFullTrajectory.Get();

            setting_render_display3D = settings_show3D.Get();
            setting_render_displayProcess = settings_showLiveProcess.Get();
            setting_render_displayVideo = settings_showLiveVideo.Get();


            if (settings_resetButton.Get()) {
                printf("RESET!\n");
                settings_resetButton.Reset();
                setting_render_fullResetRequested = true;
            }

            // Swap frames and Process Events
            pangolin::FinishFrame();


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
/*	model3DMutex.lock();
	for(size_t i=0; i<keyframes.size();i++) delete keyframes[i];
	keyframes.clear();
	allFramePoses.clear();
	keyframesByKFID.clear();
	connections.clear();
	model3DMutex.unlock();*/


        openImagesMutex.lock();
        videoImg->setBlack();
        processImg->setBlack();
        videoImgChanged = processImgChanged = true;
        openImagesMutex.unlock();

        needReset = false;
    }

    void PangolinViewer::publishCamPose(Eigen::Matrix4d &cam_pose) {
        if (!setting_render_display3D) return;
        if (!setting_render_showCurrentCamera) return;

        std::unique_lock<std::mutex> lk(model3DMutex);
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        lastNcamPoseMs.push_back(
                ((time_now.tv_sec - last_camPose_t.tv_sec) * 1000.0f + (time_now.tv_usec - last_camPose_t.tv_usec) / 1000.0f));
        if (lastNcamPoseMs.size() > 10) lastNcamPoseMs.pop_front();
        last_camPose_t = time_now;

//    currentCam->setFromF(frame, HCalib);
//    allFramePoses.push_back(frame->camToWorld.translation().cast<float>());
        camToWorld = cam_pose.cast<float>();
        allFramePoses.emplace_back(camToWorld.block<3, 1>(0, 3));
    }

    void PangolinViewer::publishImuPose(Eigen::Matrix4d &imu_pose) {
        if (!setting_render_display3D) return;
        if (!setting_render_showCurrentImu) return;

        std::unique_lock<std::mutex> lk(model3DMutex);
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        lastNimuPoseMs.push_back(
                ((time_now.tv_sec - last_imuPose_t.tv_sec) * 1000.0f + (time_now.tv_usec - last_imuPose_t.tv_usec) / 1000.0f));
        if (lastNimuPoseMs.size() > 30) lastNimuPoseMs.pop_front();
        last_imuPose_t = time_now;

        imuToWorld = imu_pose.cast<float>();
    }

    void PangolinViewer::publishVideoImg(cv::Mat video_img) {
        if (!setting_render_displayVideo) return;

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

/*        struct timeval time0;
        gettimeofday(&time0, NULL);*/

        if (rgb.depth() == CV_16U) {
            typedef cv::Point3_<uint16_t> Pixel;
            rgb.forEach<Pixel>([this](Pixel &p, const int *position) -> void {
                int idx = position[1] + position[0] * this->w;
                videoImg->data[idx][0] = p.x / 256;
                videoImg->data[idx][1] = p.y / 256;
                videoImg->data[idx][2] = p.z / 256;
            });
        } else if (rgb.depth() == CV_8U) {
            typedef cv::Point3_<uint8_t> Pixel;
            rgb.forEach<Pixel>([this](Pixel &p, const int *position) -> void {
                int idx = position[1] + position[0] * this->w;
                videoImg->data[idx][0] = p.x;
                videoImg->data[idx][1] = p.y;
                videoImg->data[idx][2] = p.z;
            });
        } else {
            LOG(ERROR) << "publishVideoImg: rgb img incorrect data type!";
            return;
        }

/*        struct timeval time1;
        gettimeofday(&time1, NULL);
        LOG_FIRST_N(INFO, 20) << "forEach function costs " <<
            (time1.tv_sec - time0.tv_sec) * 1000.0f + (time1.tv_usec - time0.tv_usec) / 1000.0f;*/

        // Equally effective method for CV_16U.
/*        cv::Mat rgb888;
        rgb.convertTo(rgb888, CV_8U, 1.0/256);
        memcpy(videoImg->data, rgb888.data, w*h*3);

        struct timeval time2;
        gettimeofday(&time2, NULL);
        LOG_FIRST_N(INFO, 20) << "convertTo and memcpy function costs " <<
            (time2.tv_sec - time1.tv_sec) * 1000.0f + (time2.tv_usec - time1.tv_usec) / 1000.0f;*/

        videoImgChanged = true;
    }

    void PangolinViewer::publishProcessImg(cv::Mat process_img) {
        if (!setting_render_displayProcess) return;

        cv::Mat rgb;
        if (process_img.channels() == 3) {
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

        if (rgb.depth() == CV_16U) {
            typedef cv::Point3_<uint16_t> Pixel;
            rgb.forEach<Pixel>([this](Pixel &p, const int *position) -> void {
                int idx = position[1] + position[0] * this->w;
                processImg->data[idx][0] = p.x / 256;
                processImg->data[idx][1] = p.y / 256;
                processImg->data[idx][2] = p.z / 256;
            });
        } else if (rgb.depth() == CV_8U) {
            typedef cv::Point3_<uint8_t> Pixel;
            rgb.forEach<Pixel>([this](Pixel &p, const int *position) -> void {
                int idx = position[1] + position[0] * this->w;
                processImg->data[idx][0] = p.x;
                processImg->data[idx][1] = p.y;
                processImg->data[idx][2] = p.z;
            });
        } else {
            LOG(ERROR) << "publishprocessImg: rgb img incorrect data type!";
            return;
        }

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

    void PangolinViewer::drawImu(float lineWidth, float *color, float sizeFactor) {
        if (!setting_render_showCurrentImu)
            return;

        int width = 640, height = 480;
        float fx = 460, fy = 460, cx = width / 2.0, cy = height / 2.0;
        float sz = sizeFactor;

        glPushMatrix();
//    Sophus::Matrix4f m = camToWorld.matrix().cast<float>();
        glMultMatrixf((GLfloat *) imuToWorld.data());


/*        if (color == 0) {
            glColor3f(1, 0, 0);
        } else
            glColor3f(color[0], color[1], color[2]);*/
        glColor3f(1, 0, 0);

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

    void PangolinViewer::drawCam(float lineWidth, float *color, float sizeFactor) {
        if (!setting_render_showCurrentCamera)
            return;

        int width = 640, height = 480;
        float fx = 460, fy = 460, cx = width / 2.0, cy = height / 2.0;
        float sz = sizeFactor;

        glPushMatrix();
//    Sophus::Matrix4f m = camToWorld.matrix().cast<float>();
        glMultMatrixf((GLfloat *) camToWorld.data());

/*        if (color == 0) {
            glColor3f(0, 0, 1);
        } else
            glColor3f(color[0], color[1], color[2]);*/
        glColor3f(0, 0, 1);

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




}

