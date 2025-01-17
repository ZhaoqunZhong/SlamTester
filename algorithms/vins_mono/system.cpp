#include "system.h"
#include "VinsAlgorithm.h"

using namespace std;
using namespace cv;

System::System(string sConfig_file_, VinsAlgorithm *interface)
        :bStart_backend(true), process_exited(false)
{
    vins_estimator::readParameters(sConfig_file_);
    feature_tracker::readParameters(sConfig_file_);
    trackerData[0].readIntrinsicParameter(sConfig_file_);
    estimator.setParameter();
    // estimator.clearState();
/*    ofs_pose.open("./pose_output.txt",fstream::out);
    if(!ofs_pose.is_open())
    {
        cerr << "ofs_pose is not open" << endl;
    }*/
    // thread thd_RunBackend(&System::process,this);
    // thd_RunBackend.detach();
    algoInterface = interface;
}

System::~System()
{
    // bStart_backend = false;

    m_buf.lock();
    while (!feature_buf.empty())
        feature_buf.pop();
    while (!imu_buf.empty())
        imu_buf.pop();
    m_buf.unlock();

    m_estimator.lock();
    estimator.clearState();
    m_estimator.unlock();

    // ofs_pose.close();
}

void System::subImageData(double dStampSec, Mat &img)
{
    if (!init_feature)
    {
        LOG(INFO) << "1 PubImageData skip the first detected feature, which doesn't contain optical flow speed" << endl;
        init_feature = 1;
        return;
    }

    if (first_image_flag)
    {
        LOG(INFO) << "2 PubImageData first_image_flag" << endl;
        first_image_flag = false;
        first_image_time = dStampSec;
        last_image_time = dStampSec;
        return;
    }
    // detect unstable camera stream
    if (dStampSec - last_image_time > 1.0 || dStampSec < last_image_time)
    {
        LOG(ERROR) << "3 PubImageData image discontinue! reset the feature tracker!" << endl;
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        return;
    }
    last_image_time = dStampSec;
    // frequency control
    if (round(1.0 * pub_count / (dStampSec - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (dStampSec - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = dStampSec;
            pub_count = 0;
        }
    }
    else
    {
        PUB_THIS_FRAME = false;
    }

    // TicToc t_r;
    // cout << "3 PubImageData t : " << dStampSec << endl;

    trackerData[0].readImage(img, dStampSec);

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        completed |= trackerData[0].updateID(i);

        if (!completed)
            break;
    }
    if (PUB_THIS_FRAME)
    {
        pub_count++;
        shared_ptr<IMG_MSG> feature_points(new IMG_MSG());
        feature_points->header = dStampSec;
        vector<set<int>> hash_ids(feature_tracker::NUM_OF_CAM);
        for (int i = 0; i < feature_tracker::NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
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
            //}
            // skip the first image; since no optical speed on frist image
            if (!init_pub)
            {
                cout << "4 PubImage init_pub skip the first image!" << endl;
                init_pub = 1;
            }
            else
            {
                m_buf.lock();
                feature_buf.push(feature_points);
                // cout << "5 PubImage t : " << fixed << feature_points->header
                //     << " feature_buf size: " << feature_buf.size() << endl;
                m_buf.unlock();
                con.notify_one();
            }
        }
    }


    cv::Mat show_img;
    cv::cvtColor(img, show_img, COLOR_GRAY2RGB);

    if (SHOW_TRACK)
    {
        for (unsigned int j = 0; j < trackerData[0].cur_pts.size(); j++)
        {
            double len = min(1.0, 1.0 * trackerData[0].track_cnt[j] / vins_estimator::WINDOW_SIZE);
            cv::circle(show_img, trackerData[0].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
        }

/*        cv::namedWindow("IMAGE", WINDOW_AUTOSIZE);
        cv::imshow("IMAGE", show_img);
        cv::waitKey(1);*/
        // updatePreviewMat(show_img, true);

        for (auto &oi : algoInterface->output_interfaces) {
            oi->publishProcessImg(show_img);
        }
    }

    // cout << "5 PubImage" << endl;

}

vector<pair<vector<ImuConstPtr>, ImgConstPtr>> System::getMeasurements()
{
    vector<pair<vector<ImuConstPtr>, ImgConstPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
        {
            // cerr << "1 imu_buf.empty() || feature_buf.empty()" << endl;
            return measurements;
        }

        if (!(imu_buf.back()->header > feature_buf.front()->header + estimator.td))
        {
            LOG(WARNING) << "wait for imu, only should happen at the beginning sum_of_wait: "
                 << sum_of_wait << endl;
            sum_of_wait++;
            return measurements;
        }

        if (!(imu_buf.front()->header < feature_buf.front()->header + estimator.td))
        {
            LOG(WARNING) << "throw img, only should happen at the beginning" << endl;
            feature_buf.pop();
            continue;
        }
        ImgConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        vector<ImuConstPtr> IMUs;
        while (imu_buf.front()->header < img_msg->header + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        // cout << "1 getMeasurements IMUs size: " << IMUs.size() << endl;
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty()){
            LOG(ERROR) << "no imu between two image" << endl;
        }
        // cout << "1 getMeasurements img t: " << fixed << img_msg->header
        //     << " imu begin: "<< IMUs.front()->header
        //     << " end: " << IMUs.back()->header
        //     << endl;
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

void System::subImuData(double dStampSec, const Eigen::Vector3d &vGyr,
                        const Eigen::Vector3d &vAcc)
{
    shared_ptr<IMU_MSG> imu_msg(new IMU_MSG());
    imu_msg->header = dStampSec;
    imu_msg->linear_acceleration = vAcc;
    imu_msg->angular_velocity = vGyr;

    if (dStampSec <= last_imu_t)
    {
        cerr << "imu message in disorder!" << endl;
        return;
    }
    last_imu_t = dStampSec;
    // cout << "1 PubImuData t: " << fixed << imu_msg->header
    //     << " acc: " << imu_msg->linear_acceleration.transpose()
    //     << " gyr: " << imu_msg->angular_velocity.transpose() << endl;
    m_buf.lock();
    imu_buf.push(imu_msg);
    // cout << "1 PubImuData t: " << fixed << imu_msg->header
    //     << " imu_buf size:" << imu_buf.size() << endl;
    m_buf.unlock();
    con.notify_one();
}

// thread: visual-inertial odometry
void System::process()
{
    // cout << "1 ProcessBackEnd start" << endl;
    while (bStart_backend)
    {
        // cout << "1 process()" << endl;
        vector<pair<vector<ImuConstPtr>, ImgConstPtr>> measurements;

        unique_lock<mutex> lk(m_buf);
        con.wait(lk, [&] {
            if (!bStart_backend) {
                return true;
            }
            return (measurements = getMeasurements()).size() != 0;
        });
        if( measurements.size() > 1){
/*            cout << "1 getMeasurements size: " << measurements.size()
                 << " imu sizes: " << measurements[0].first.size()
                 << " feature_buf size: " <<  feature_buf.size()
                 << " imu_buf size: " << imu_buf.size() << endl;*/
        }
        lk.unlock();
        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)
            {
                double t = imu_msg->header;
                double img_t = img_msg->header + estimator.td;
                if (t <= img_t)
                {
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
                }
                else
                {
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
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
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

            if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            {
/*                Vector3d p_wi;
                Quaterniond q_wi;
                q_wi = Quaterniond(estimator.Rs[vins_estimator::WINDOW_SIZE]);
                p_wi = estimator.Ps[vins_estimator::WINDOW_SIZE];*/
                Matrix4d imu_to_world = Matrix4d::Identity();
                imu_to_world.block<3,3>(0,0) = estimator.Rs[vins_estimator::WINDOW_SIZE];
                imu_to_world.block<3,1>(0,3) = estimator.Ps[vins_estimator::WINDOW_SIZE];
                Matrix4d world_to_imu = imu_to_world.inverse();
                for (auto &oi : algoInterface->output_interfaces) {
                    oi->publishImuPose(world_to_imu, estimator.Headers[vins_estimator::WINDOW_SIZE]);
                    Matrix4d world_to_cam = world_to_imu * algoInterface->input_interfaces[0]->camToImu.inverse();
                    oi->publishCamPose(world_to_cam, estimator.Headers[vins_estimator::WINDOW_SIZE]);
                }
/*                vPath_to_draw.push_back(p_wi);
                double dStamp = estimator.Headers[vins_estimator::WINDOW_SIZE];
                cout << "1 BackEnd processImage dt: " << fixed << t_processImage.toc() << " stamp: " <<  dStamp << " p_wi: " << p_wi.transpose() << endl;
                ofs_pose << fixed << dStamp << " " << p_wi(0) << " " << p_wi(1) << " " << p_wi(2) << " "
                         << q_wi.w() << " " << q_wi.x() << " " << q_wi.y() << " " << q_wi.z() << endl;*/
/*                updatePoseForDrawing(p_wi, q_wi);
                /// display sliding window
                pthread_mutex_lock(&pose_mtx);
                key_frames.clear();
                for (int i = 0; i < vins_estimator::WINDOW_SIZE + 1; i++) {
                    q_wi = Quaterniond(estimator.Rs[i]);
                    p_wi = estimator.Ps[i];
                    Eigen::Matrix4d T_wi = Eigen::Matrix4d::Identity();
                    T_wi.block<3,3>(0,0) = q_wi.toRotationMatrix();
                    T_wi.block<3,1>(0,3) = p_wi;
                    key_frames.push_back(T_wi);
                }
                pthread_mutex_unlock(&pose_mtx);*/
            }
        }
        m_estimator.unlock();
    }

    process_exited = true;
}

