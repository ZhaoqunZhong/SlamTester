//
// Created by 仲钊群 on 2022/10/9.
//

#include "TumRs_VinsMono.h"
#include "yaml-cpp/yaml.h"
#include "glog/logging.h"


TumRsVinsMono::TumRsVinsMono(std::string &cam_config, std::string &imu_config, std::string &ci_extrinsic, std::string &ros_bag) {
    data_bag = ros_bag;

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

    bag_topics = {monoImg_topic, imu_topic, acc_topic, gyr_topic};

    // getUndistorterFromFile(cam_config, "", "");
}

VinsMonoAlgorithm::VinsMonoAlgorithm(std::string &vins_config) : bStart_backend(true), process_exited(false) {
    FLAGS_logtostdout = true;
    FLAGS_colorlogtostdout = true;

    estimator.config_path_ = vins_config;
    vins_estimator::readParameters(vins_config);
    feature_tracker::readParameters(vins_config);
    trackerData[0].readIntrinsicParameter(vins_config);
    estimator.setParameter();
}

VinsMonoAlgorithm::~VinsMonoAlgorithm() {
    m_buf.lock();
    while (!feature_buf.empty())
        feature_buf.pop();
    while (!imu_buf.empty())
        imu_buf.pop();
    m_buf.unlock();

    m_estimator.lock();
    estimator.clearState();
    m_estimator.unlock();
}
#if 1
void VinsMonoAlgorithm::feedMonoImg(double ts, cv::Mat mono) {
    if (mono.channels() != 1) {
        LOG(ERROR) << "feedMonoImg input not mono.";
        return;
    }
    if (mono.depth() == CV_16U)
        mono.convertTo(mono, CV_8U, 1.0/256);
    else if (mono.depth() != CV_8U && mono.depth() != CV_32F) {
        LOG(ERROR) << "Opencv goodFeatureToTrack doesn't work with this data type.";
        return;
    }

    if (first_image_flag) {
        LOG(INFO) << "subImageData first_image_flag" << endl;
        first_image_flag = false;
        last_publish_time = -1;
        last_image_time = ts;
        // cv::imwrite("/Users/zhongzhaoqun/Downloads/dataset-seq1/vins/feedMonoImg.png", mono);
        trackerData[0].readImage(mono, ts);
        return;
    }
    // detect unstable camera stream
    if (ts - last_image_time > 1.0 || ts < last_image_time) {
        LOG(ERROR) << "subImageData image discontinue! reset the feature tracker!" << endl;
        first_image_flag = true;
        last_image_time = 0;
        return;
    }
    last_image_time = ts;

    // frequency control
    if (ts - last_publish_time >= 0.999 / FREQ) {
        PUB_THIS_FRAME = true;
        last_publish_time = ts;
    } else
        PUB_THIS_FRAME = false;

    trackerData[0].readImage(mono, ts);

    shared_ptr<IMG_MSG> feature_points(new IMG_MSG());
    feature_points->header = ts;
    for (int i = 0; i < feature_tracker::NUM_OF_CAM; i++) {
        auto &un_pts = trackerData[i].cur_un_pts;
        auto &cur_pts = trackerData[i].cur_pts;
        auto &ids = trackerData[i].ids;
        auto &pts_velocity = trackerData[i].pts_velocity;
        for (unsigned int j = 0; j < ids.size(); j++) {
            if (trackerData[i].track_cnt[j] > 1) {
                int p_id = ids[j];
                double x = un_pts[j].x;
                double y = un_pts[j].y;
                double z = 1;
                feature_points->points.push_back(Vector3d(x, y, z));
                feature_points->id_of_point.push_back(p_id * feature_tracker::NUM_OF_CAM + i);
                feature_points->u_of_point.push_back(cur_pts[j].x);
                feature_points->v_of_point.push_back(cur_pts[j].y);
                feature_points->velocity_x_of_point.push_back(pts_velocity[j].x);
                feature_points->velocity_y_of_point.push_back(pts_velocity[j].y);
            }
        }
    }
    if (estimator.solver_flag == Estimator::NON_LINEAR) {
        mo_buf_mtx_.lock();
        mo_img_buf_.push(feature_points);
        mo_buf_mtx_.unlock();
        mo_buf_con_.notify_one();
    }
    if (PUB_THIS_FRAME) {
        //Features guarantee to have velocity now
        m_buf.lock();
        feature_buf.push(feature_points);
        m_buf.unlock();
        con.notify_one();
    }

    /// Feature tracking visualization
    if (!output_interfaces.empty()) {
        auto f = [](uint64_t ts, cv::Mat mat, VinsMonoAlgorithm *sys) {
            cv::Mat show_img;
            cv::cvtColor(mat, show_img, cv::COLOR_GRAY2RGB);
            for (unsigned int j = 0; j < sys->trackerData[0].cur_pts.size(); j++) {
                double len = min(1.0, 1.0 * sys->trackerData[0].track_cnt[j] /
                                      vins_estimator::WINDOW_SIZE);
                cv::circle(show_img, sys->trackerData[0].cur_pts[j], 5,
                           cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
            }
            for (auto &oi: sys->output_interfaces) {
                oi->publishVideoImg(mat);
                oi->publishProcessImg(show_img);
                // cv::imwrite("/Users/zhongzhaoqun/Downloads/dataset-seq1/vins/publishProcessImg.png", show_img);
            }
        };
        uint64_t image_ts = ts * 1e9;
        std::thread t(f, std::ref(image_ts), mono.clone(), this);
        t.detach();
    }
}

#else
void VinsMonoAlgorithm::feedMonoImg(double ts, cv::Mat mono) {
    if (mono.channels() != 1) {
        LOG(ERROR) << "feedMonoImg input not mono.";
        return;
    }
    if (mono.depth() == CV_16U)
        mono.convertTo(mono, CV_8U, 1.0/256);
    else if (mono.depth() != CV_8U && mono.depth() != CV_32F) {
        LOG(ERROR) << "Opencv goodFeatureToTrack doesn't work with this data type.";
        return;
    }
    std::vector<cv::Point2f> features;
    cv::goodFeaturesToTrack(mono, features, 150, 0.01, 30);
    cv::Mat show_img;
    cv::cvtColor(mono, show_img, cv::COLOR_GRAY2RGB);
    for (unsigned int j = 0; j < features.size(); j++) {
        double len = 0.5;
        cv::circle(show_img, features[j], 5,
                   cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    for (auto &oi: output_interfaces) {
        oi->publishVideoImg(mono);
        oi->publishProcessImg(show_img);
    }
}
#endif

void VinsMonoAlgorithm::feedImu(double ts, Eigen::Vector3d acc, Eigen::Vector3d gyr) {
    for (auto &oi: output_interfaces) {
        oi->publishImuMsg(acc, gyr);
    }

    shared_ptr<IMU_MSG> imu_msg(new IMU_MSG());
    imu_msg->header = ts;
    imu_msg->linear_acceleration = acc;
    imu_msg->angular_velocity = gyr;

    if (ts <= last_imu_t) {
        LOG(ERROR) << "imu message in disorder!" << endl;
        return;
    }
    last_imu_t = ts;

    // TimeLagMeasurer timer;
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    // con.notify_one();
    // double t = timer.lagFromStartSecond()*1e3;
    // LOG_IF(INFO, t > 1.5) << "subImuData cost " << t << " ms";

    if (estimator.solver_flag == Estimator::NON_LINEAR) {
        mo_buf_mtx_.lock();
        mo_imu_buf_.push(imu_msg);
        mo_buf_mtx_.unlock();
    }
}


std::vector<pair<vector<ImuConstPtr>, ImgConstPtr>> VinsMonoAlgorithm::getMeasurements() {
    std::vector<pair<vector<ImuConstPtr>, ImgConstPtr>> measurements;

    while (true) {
        if (imu_buf.empty() || feature_buf.empty()) {
            // LOG(WARNING) << "imu_buf.empty() || feature_buf.empty()" << endl;
            return measurements;
        }

        if (imu_buf.back()->header <= feature_buf.front()->header + estimator.td) {
            LOG(WARNING) << "wait for imu, only should happen at the beginning, sum_of_wait: " << sum_of_wait << endl;
            sum_of_wait++;
            return measurements;
        }

        if (imu_buf.front()->header >= feature_buf.front()->header + estimator.td) {
            LOG(WARNING) << "throw img, only should happen at the beginning" << endl;
            feature_buf.pop();
            continue;
        }
        ImgConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        vector<ImuConstPtr> IMUs;
        while (imu_buf.front()->header < img_msg->header + estimator.td) {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }

        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty()) {
            LOG(ERROR) << "no imu between two image" << endl;
        }

        measurements.emplace_back(IMUs, img_msg);
        // LOG(INFO) << "remaining imu num " << imu_buf.size() << " ";
    }

    // return measurements;
}


// thread: visual-inertial odometry
void VinsMonoAlgorithm::process() {
    while (bStart_backend) {
        vector<pair<vector<ImuConstPtr>, ImgConstPtr>> measurements;

        unique_lock<mutex> lk(m_buf);
        con.wait(lk, [&] {
            if (!bStart_backend) {
                return true;
            }
            return !(measurements = getMeasurements()).empty();
        });
        lk.unlock();

        m_estimator.lock();
        LOG_IF(WARNING, measurements.size() > 1) << "measurement size " << measurements.size() << " ";
        for (auto &measurement: measurements) {
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg: measurement.first) {
                double t = imu_msg->header;
                double img_t = img_msg->header + estimator.td;
                if (t <= img_t) {
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    assert(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x();
                    dy = imu_msg->linear_acceleration.y();
                    dz = imu_msg->linear_acceleration.z();
                    rx = imu_msg->angular_velocity.x();
                    ry = imu_msg->angular_velocity.y();
                    rz = imu_msg->angular_velocity.z();
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    // printf("1 BackEnd imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                } else {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    assert(dt_1 >= 0);
                    assert(dt_2 >= 0);
                    assert(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x();
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y();
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z();
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x();
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y();
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z();
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }

            // cout << "processing vision data with stamp:" << img_msg->header
            //     << " img_msg->points.size: "<< img_msg->points.size() << endl;

            // TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++) {
                int v = img_msg->id_of_point[i] + 0.5;
                int feature_id = v / feature_tracker::NUM_OF_CAM;
                int camera_id = v % feature_tracker::NUM_OF_CAM;
                double x = img_msg->points[i].x();
                double y = img_msg->points[i].y();
                double z = img_msg->points[i].z();
                double p_u = img_msg->u_of_point[i];
                double p_v = img_msg->v_of_point[i];
                double velocity_x = img_msg->velocity_x_of_point[i];
                double velocity_y = img_msg->velocity_y_of_point[i];
                assert(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id, xyz_uv_velocity);
            }
            vins_estimator::TicToc t_processImage;
            estimator.processImage(image, img_msg->header);

            /// save or visualize result so far
            if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR) {
                /// display current pose
/*                Vector3d p_wi;
                Quaterniond q_wi;
                q_wi = Quaterniond(estimator.Rs[vins_estimator::WINDOW_SIZE]);
                p_wi = estimator.Ps[vins_estimator::WINDOW_SIZE];
                updatePoseForDrawing(p_wi, q_wi);*/
                Eigen::Matrix4d world_to_imu;
                world_to_imu.setIdentity();
                world_to_imu.block<3,3>(0,0) = estimator.Rs[vins_estimator::WINDOW_SIZE];
                world_to_imu.block<3,1>(0,3) = estimator.Ps[vins_estimator::WINDOW_SIZE];
                Eigen::Matrix4d imu_to_world = world_to_imu.inverse();
                for (auto &oi: output_interfaces) {
                    oi->publishImuPose(imu_to_world);
                }
            }
        }
        m_estimator.unlock();
    }

    process_exited = true;
}


std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> VinsMonoAlgorithm::getMoMeasurements() {
    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> measurements;

    estimator.mo_estimator_.updateFrontend(Rwbs, Pwbs, Vb_end, Ba_end, Bg_end, ts_end, td, feature_end);

    if (ts_end < 0) {
        std::queue<ImgConstPtr> empty_img_buf;
        std::queue<ImuConstPtr> empty_imu_buf;
        swap(empty_img_buf, mo_img_buf_);
        swap(empty_imu_buf, mo_imu_buf_);
        return measurements;
    }
    while (!mo_imu_buf_.empty() && mo_imu_buf_.front()->header <= ts_end + td) {
        mo_imu_buf_.pop();
    }
    while (!mo_img_buf_.empty() && mo_img_buf_.front()->header <= ts_end) {
        mo_img_buf_.pop();
    }

    auto imu_buf_copy = mo_imu_buf_;
    while (true) {
        if (mo_imu_buf_.empty() || mo_img_buf_.empty()) {
            return measurements;
        }
        if (imu_buf_copy.back()->header <= mo_img_buf_.front()->header + td) {
            LOG(WARNING) << "getMoMeasurements() wait for imu.";
            return measurements;
        }
        if (imu_buf_copy.front()->header >= mo_img_buf_.front()->header + td) {
            /*            LOG(WARNING) << "getMoMeasurements() throw image.";
                        LOG(INFO) << "ts_end " << to_string(ts_end) << " ";
                        LOG(INFO) << "imu_buf_copy.front()->header " << to_string(imu_buf_copy.front()->header) << " ";
                        LOG(INFO) << "mo_img_buf_.front()->header + td " << to_string(mo_img_buf_.front()->header + td) << " ";*/
            mo_img_buf_.pop();
            continue;
        }
        ImgConstPtr img_msg = mo_img_buf_.front();
        mo_img_buf_.pop();

        vector<ImuConstPtr> IMUs;
        while (imu_buf_copy.front()->header < img_msg->header + td) {
            IMUs.emplace_back(imu_buf_copy.front());
            imu_buf_copy.pop();
        }
        IMUs.emplace_back(imu_buf_copy.front());
        if (IMUs.empty()) {
            LOG(ERROR) << "no imu between two image" << endl;
        }

        measurements.emplace_back(IMUs, img_msg);
    }
}

void VinsMonoAlgorithm::motionOnlyProcess() {
    while (mo_estimate_start) {
        std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> measurements;
        unique_lock<mutex> lk(mo_buf_mtx_);
        mo_buf_con_.wait(lk, [&] {
            if (!mo_estimate_start) {
                return true;
            }
            return !(measurements = getMoMeasurements()).empty();
        });
        if (!mo_estimate_start)
            break;

        lk.unlock();
        // LOG(INFO) << "measurements size " << measurements.size() << "------------- ";
        ceres::Problem problem;
        ceres::LossFunction *loss_function;
        // loss_function = new ceres::HuberLoss(1.0);
        loss_function = new ceres::CauchyLoss(1.0);
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        vector<Vector3d> Vs;
        vector<std::array<double, 7>> poses;
        vector<shared_ptr<IntegrationBase>> imu_integrations;
        vector<ImgConstPtr> frames;
        Matrix3d last_R = Rwbs[vins_estimator::WINDOW_SIZE];
        Vector3d last_P = Pwbs[vins_estimator::WINDOW_SIZE], last_V = Vb_end;
        std::array<double, 7> pose;
        pose[0] = Pwbs[vins_estimator::WINDOW_SIZE].x();
        pose[1] = Pwbs[vins_estimator::WINDOW_SIZE].y();
        pose[2] = Pwbs[vins_estimator::WINDOW_SIZE].z();
        Quaterniond q(Rwbs[vins_estimator::WINDOW_SIZE]);
        pose[3] = q.x();
        pose[4] = q.y();
        pose[5] = q.z();
        pose[6] = q.w();
        poses.push_back(pose);
        Vs.push_back(Vb_end);
        for (auto &measurement : measurements) {
            /*            for (auto & imu : measurement.first) {
                            LOG(INFO) << "imu ts " << to_string(imu->header) << " ";
                        }
                        LOG(INFO) << "image ts + td " << to_string(measurement.second->header + td) << " ";*/
            Matrix3d R = last_R;
            Vector3d P = last_P, V = last_V;
            auto &imu0 = measurement.first.front();
            shared_ptr<IntegrationBase> pre_integration = make_shared<IntegrationBase>(imu0->linear_acceleration,
                                                                                       imu0->angular_velocity, Ba_end,
                                                                                       Bg_end);
            acc_0 = imu0->linear_acceleration;
            gyr_0 = imu0->angular_velocity;
            double last_t, dt;

            double img_t = measurement.second->header + td;
            frames.push_back(measurement.second);

            for (auto &imu : measurement.first) {
                double t = imu->header;
                if (t <= img_t) {
                    if (init_imu) {
                        init_imu = false;
                        dt = t - (ts_end + td);
                    } else
                        dt = imu->header - last_t;
                    last_t = t;
                } else {
                    dt = img_t - last_t;
                    last_t = img_t;
                }
                pre_integration->push_back(dt, imu->linear_acceleration, imu->angular_velocity);

                Vector3d un_acc_0 = R * (acc_0 - Ba_end) - estimator.g;
                Vector3d un_gyr = 0.5 * (gyr_0 + imu->angular_velocity) - Bg_end;
                R *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
                Vector3d un_acc_1 = R * (imu->linear_acceleration - Ba_end) - estimator.g;
                Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
                P += dt * V + 0.5 * dt * dt * un_acc;
                V += dt * un_acc;

                acc_0 = imu->linear_acceleration;
                gyr_0 = imu->angular_velocity;
            }
            imu_integrations.push_back(pre_integration);

            Vs.push_back(V);
            std::array<double, 7> pose;
            pose[0] = P.x();
            pose[1] = P.y();
            pose[2] = P.z();
            Quaterniond q(R);
            pose[3] = q.x();
            pose[4] = q.y();
            pose[5] = q.z();
            pose[6] = q.w();
            poses.push_back(pose);

            last_P = P;
            last_R = R;
            last_V = V;
            init_imu = true;
        }

        for (auto &pose : poses) {
            problem.AddParameterBlock(pose.data(), 7, local_parameterization);
        }
        problem.SetParameterBlockConstant(poses[0].data());
        for (auto &v : Vs) {
            problem.AddParameterBlock(v.data(), 3);
        }
        problem.SetParameterBlockConstant(Vs[0].data());

        uint cnt = 0;
        for (int i = 0; i < imu_integrations.size(); i++) {
            MotionOnlyIMUFactor *imu_factor = new MotionOnlyIMUFactor(imu_integrations[i], Ba_end, Bg_end);
            problem.AddResidualBlock(imu_factor, NULL, poses[i].data(), Vs[i].data(), poses[i + 1].data(),
                                     Vs[i + 1].data());

            for (int j = 0; j < frames[i]->points.size(); j++) {
                int feature_id = frames[i]->id_of_point[j];
                unordered_map<int, Vector3d>::iterator feature = feature_end.find(feature_id);
                if (feature != feature_end.end()) {
                    Vector3d norm_coord = frames[i]->points[j];
                    MotionOnlyProjectionFactor *f = new MotionOnlyProjectionFactor(feature->second, norm_coord);
                    problem.AddResidualBlock(f, loss_function, poses[i + 1].data());
                    cnt++;
                }
            }
        }
        LOG(INFO) << "number of feature projections " << cnt << " ";

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.trust_region_strategy_type = ceres::DOGLEG;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        LOG(INFO) << "Motion only ba result " << summary.BriefReport();

        std::array<double, 7> latest_pose = poses.back();
        Vector3d p_latest(latest_pose[0], latest_pose[1], latest_pose[2]);
        Quaterniond q_lastest(latest_pose[6], latest_pose[3], latest_pose[4], latest_pose[5]);
        // updatePoseForDrawing(p_latest, q_lastest);
    }

    mo_estimate_exited = true;
}

void VinsMonoAlgorithm::start() {
    vi_th_ = std::thread(&VinsMonoAlgorithm::process, this);
    vi_th_.detach();
/*    mo_th_ = std::thread(&VinsMonoAlgorithm::motionOnlyProcess, this);
    mo_th_.detach();*/
}

void VinsMonoAlgorithm::stop() {
    bStart_backend = false;
    // mo_estimate_start = false;
    // mo_buf_con_.notify_one();
    while (!process_exited /*|| !mo_estimate_exited*/) {
        con.notify_one();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
