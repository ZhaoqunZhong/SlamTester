//
// Created by 仲钊群 on 2022/10/13.
//

#include "SlamInterface.h"
#include "glog/logging.h"
#include <fstream>

DECLARE_bool(resizeAndUndistort);

namespace SlamTester {

    void InputInterface::getUndistorterFromFile(std::string configFilename, std::string gammaFilename,
                                                    std::string vignetteFilename) {
        printf("Reading Calibration from file %s", configFilename.c_str());

        std::ifstream f(configFilename.c_str());
        if (!f.good()) {
            f.close();
            printf(" ... not found. Cannot operate without calibration, shutting down.\n");
            return;
        }

        printf(" ... found!\n");
        std::string l1, l2, l3, l4;
        std::getline(f, l1);
        std::getline(f, l2);
        std::getline(f, l3);
        std::getline(f, l4);
        f.close();

        // l2 && l4
        LOG_ASSERT(std::sscanf(l2.c_str(), "%d %d", &orig_w, &orig_h) == 2);
        LOG_ASSERT(std::sscanf(l4.c_str(), "%d %d", &inner_w, &inner_h) == 2);

        double ic[10];
        // l1
        if (std::sscanf(l1.c_str(), "KannalaBrandt %lf %lf %lf %lf %lf %lf %lf %lf",
                             &ic[0], &ic[1], &ic[2], &ic[3],
                             &ic[4], &ic[5], &ic[6], &ic[7]) == 8) {
            printf("found KannalaBrandt camera model, building rectifier.\n");
            distort_model = KannalaBrandt;
            cam_distort = {ic[4], ic[5], ic[6], ic[7]};
        } else if (std::sscanf(l1.c_str(), "RadTan %lf %lf %lf %lf %lf %lf %lf %lf",
                               &ic[0], &ic[1], &ic[2], &ic[3],
                               &ic[4], &ic[5], &ic[6], &ic[7]) == 8) {
            printf("found RadTan camera model, building rectifier.\n");
            distort_model = RadTan;
            cam_distort = {ic[4], ic[5], ic[6], ic[7]};
        }else if (std::sscanf(l1.c_str(), "RadTan %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                              &ic[0], &ic[1], &ic[2], &ic[3],
                              &ic[4], &ic[5], &ic[6], &ic[7], &ic[8]) == 9) {
            printf("found RadTan camera model, building rectifier.\n");
            distort_model = RadTan;
            cam_distort = {ic[4], ic[5], ic[6], ic[7], ic[8]};
        } else if (std::sscanf(l1.c_str(), "EquiDistant %lf %lf %lf %lf %lf %lf %lf %lf",
                               &ic[0], &ic[1], &ic[2], &ic[3],
                               &ic[4], &ic[5], &ic[6], &ic[7]) == 8) {
            printf("found EquiDistant camera model, building rectifier.\n");
            distort_model = EquiDistant;
            cam_distort = {ic[4], ic[5], ic[6], ic[7]};
        } else if (std::sscanf(l1.c_str(), "FOV %lf %lf %lf %lf %lf",
                               &ic[0], &ic[1], &ic[2], &ic[3],
                               &ic[4]) == 5) {
            printf("found FOV camera model, not implemented yet, sorry.\n");
            exit(1);
        } else if (std::sscanf(l1.c_str(), "PinHole %lf %lf %lf %lf",
                               &ic[0], &ic[1], &ic[2], &ic[3])) {
            printf("found Pinhole camera model, building rectifier.\n");
            distort_model = PinHole;
            cam_distort = {0, 0, 0, 0};
        } else {
            printf("could not read calib file! exit.");
            exit(1);
        }

        if (ic[2] < 1 && ic[3] < 1) {
            printf("\n\nFound fx=%f, fy=%f, cx=%f, cy=%f.\n I'm assuming this is the \"relative\" calibration file format,"
                   "and will rescale this by image width / height to fx=%f, fy=%f, cx=%f, cy=%f.\n\n",
                   ic[0], ic[1], ic[2], ic[3],
                   ic[0] * orig_w, ic[1] * orig_h, ic[2] * orig_w - 0.5, ic[3] * orig_h - 0.5);

            // rescale and substract 0.5 offset.
            // the 0.5 is because I'm assuming the calibration is given such that the pixel at (0,0)
            // contains the integral over intensity over [0,0]-[1,1], whereas I assume the pixel (0,0)
            // to contain a sample of the intensity ot [0,0], which is best approximated by the integral over
            // [-0.5,-0.5]-[0.5,0.5]. Thus, the shift by -0.5.
            ic[0] = ic[0] * orig_w;
            ic[1] = ic[1] * orig_h;
            ic[2] = ic[2] * orig_w - 0.5;
            ic[3] = ic[3] * orig_h - 0.5;
        }

        cam_k = {ic[0], 0, ic[2], 0, ic[1], ic[3], 0, 0, 1};


        std::vector<double> given_new_k;
        std::vector<double> empty;
        // l3
        if (l3 == "crop") {
            printf("Out: Rectify Crop\n");
            if (distort_model == RadTan || distort_model == PinHole) {
                inner_cam_k = cv::getOptimalNewCameraMatrix(cam_k, cam_distort, cv::Size2i(orig_w, orig_h), 0,
                                                            cv::Size2i(inner_w, inner_h));
                cv::initUndistortRectifyMap(cam_k, cam_distort, empty, inner_cam_k, cv::Size2i(inner_w, inner_h),
                                            CV_32FC1, remapX, remapY);
            } else if (distort_model == EquiDistant || distort_model == KannalaBrandt) {
                cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cam_k, cam_distort, cv::Size2i(orig_w, orig_h),
                                                                        empty, inner_cam_k, 0,
                                                                        cv::Size2i(inner_w, inner_h));
                cv::fisheye::initUndistortRectifyMap(cam_k, cam_distort, empty, inner_cam_k,
                                                     cv::Size2i(inner_w, inner_h), CV_32FC1, remapX, remapY);
            }

        } else if (l3 == "full") {
            printf("Out: Rectify Full\n");
            if (distort_model == RadTan || distort_model == PinHole) {
                inner_cam_k = cv::getOptimalNewCameraMatrix(cam_k, cam_distort, cv::Size2i(orig_w, orig_h), 1,
                                                            cv::Size2i(inner_w, inner_h));
                cv::initUndistortRectifyMap(cam_k, cam_distort, empty, inner_cam_k, cv::Size2i(inner_w, inner_h),
                                            CV_32FC1, remapX, remapY);
            } else if (distort_model == EquiDistant || distort_model == KannalaBrandt) {
                cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cam_k, cam_distort, cv::Size2i(orig_w, orig_h),
                                                                        empty, inner_cam_k, 1,
                                                                        cv::Size2i(inner_w, inner_h));
                cv::fisheye::initUndistortRectifyMap(cam_k, cam_distort, empty, inner_cam_k,
                                                     cv::Size2i(inner_w, inner_h), CV_32FC1, remapX, remapY);
            }
        } else if (l3 == "none") {
            printf("Out: No Rectification\n");
            inner_cam_k = cam_k;
            if (distort_model == RadTan || distort_model == PinHole) {
                cv::initUndistortRectifyMap(cam_k, cam_distort, empty, inner_cam_k, cv::Size2i(inner_w, inner_h),
                                            CV_32FC1, remapX, remapY);
            } else if (distort_model == EquiDistant || distort_model == KannalaBrandt) {
                cv::fisheye::initUndistortRectifyMap(cam_k, cam_distort, empty, inner_cam_k,
                                                     cv::Size2i(inner_w, inner_h), CV_32FC1, remapX, remapY);
            }
        } else if (std::sscanf(l3.c_str(), "%f %f %f %f %f", &given_new_k[0], &given_new_k[1],
                               &given_new_k[2], &given_new_k[3], &given_new_k[4]) == 5) {
            printf("Out: %f %f %f %f %f\n",
                   given_new_k[0], given_new_k[1], given_new_k[2], given_new_k[3],
                   given_new_k[4]);
            inner_cam_k = {given_new_k[0], 0, given_new_k[2], 0, given_new_k[1], given_new_k[3], 0, 0, 1};
            if (distort_model == RadTan || distort_model == PinHole) {
                cv::initUndistortRectifyMap(cam_k, cam_distort, empty, inner_cam_k, cv::Size2i(inner_w, inner_h),
                                            CV_32FC1, remapX, remapY);
            } else if (distort_model == EquiDistant || distort_model == KannalaBrandt) {
                cv::fisheye::initUndistortRectifyMap(cam_k, cam_distort, empty, inner_cam_k,
                                                     cv::Size2i(inner_w, inner_h), CV_32FC1, remapX, remapY);
            }
        } else {
            printf("Out: Failed to Read Output pars... not rectifying.\n");
            return;
        }

        LOG(INFO) << "Inner camera K: \n" << inner_cam_k;
    }

    void InputInterface::undistortImg(cv::Mat in, cv::Mat &out) {
        if (!FLAGS_resizeAndUndistort)
            return;
        if (distort_model == RadTan || distort_model == PinHole ||
            distort_model == EquiDistant || distort_model == KannalaBrandt) {
            cv::remap(in, out, remapX, remapY, cv::INTER_LINEAR);
        }
        else {
            LOG(ERROR) << "Selected distort model not implemented yet.";
            exit(1);
        }

    }

    // Borrowed from OpenVins Loader.cpp
    void InputInterface::loadGroundTruth(std::string &gt_file) {
        if (gt_file.empty())
            return;
        std::ifstream gtf(gt_file);
        if (!gtf.good()) {
            gtf.close();
            LOG(ERROR) << "Failed to find ground truth file.";
            return;
        }

        // Loop through each line of this file
        std::string current_line;
        while (std::getline(gtf, current_line)) {

            // Skip if we start with a comment
            if (!current_line.find("#"))
                continue;

            // Loop variables
            int i = 0;
            std::istringstream s(current_line);
            std::string field;
            Eigen::Matrix<double, 20, 1> data;

            char dlm;
            if (std::filesystem::path(gt_file).extension() == ".csv")
                dlm = ',';
            else
                dlm = ' ';
            // Loop through this line (timestamp(s) tx ty tz qx qy qz qw Pr11 Pr12 Pr13 Pr22 Pr23 Pr33 Pt11 Pt12 Pt13 Pt22 Pt23 Pt33)
            while (std::getline(s, field, dlm)) {
                // Skip if empty
                if (field.empty() || i >= data.rows())
                    continue;
                // save the data to our vector
                data(i) = std::atof(field.c_str());
                i++;
            }

            // Only a valid line if we have all the parameters
            if (i >= 20) {
                // time and pose
                gt_times_s.push_back(data(0));
                gt_poses.push_back(data.block(1, 0, 7, 1));
                // covariance values
/*                Eigen::Matrix3d c_ori, c_pos;
                c_ori << data(8), data(9), data(10), data(9), data(11), data(12), data(10), data(12), data(13);
                c_pos << data(14), data(15), data(16), data(15), data(17), data(18), data(16), data(18), data(19);
                c_ori = 0.5 * (c_ori + c_ori.transpose());
                c_pos = 0.5 * (c_pos + c_pos.transpose());
                cov_ori.push_back(c_ori);
                cov_pos.push_back(c_pos);*/
            } else if (i >= 8) {
                gt_times_s.push_back(data(0));
                gt_poses.push_back(data.block(1, 0, 7, 1));
            }
        }

        // Finally close the file
        gtf.close();

        // Error if we don't have any data
        if (gt_times_s.empty()) {
            // PRINT_ERROR(RED "[LOAD]: Could not parse any data from the file!!\n" RESET);
            // PRINT_ERROR(RED "[LOAD]: %s\n" RESET, path_traj.c_str());
            LOG(ERROR) << "Could not parse any data from gt file!!";
            std::exit(EXIT_FAILURE);
        }

        // Assert that they are all equal
        if (gt_times_s.size() != gt_poses.size()) {
            // PRINT_ERROR(RED "[LOAD]: Parsing error, pose and timestamps do not match!!\n" RESET);
            // PRINT_ERROR(RED "[LOAD]: %s\n" RESET, path_traj.c_str());
            LOG(ERROR) << "Parsing error, gt poses and timestamps do not match!!";
            std::exit(EXIT_FAILURE);
        }

        // Assert that they are all equal
/*        if (!cov_ori.empty() && (times.size() != cov_ori.size() || times.size() != cov_pos.size())) {
            PRINT_ERROR(RED "[LOAD]: Parsing error, timestamps covariance size do not match!!\n" RESET);
            PRINT_ERROR(RED "[LOAD]: %s\n" RESET, path_traj.c_str());
            std::exit(EXIT_FAILURE);
        }*/
    }

}