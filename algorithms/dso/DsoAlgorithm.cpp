//
// Created by 仲钊群 on 2022/10/31.
//

#include "DsoAlgorithm.h"
#include "opencv2/core/eigen.hpp"
#include "util/globalCalib.h"
#include "src/FullSystem/FullSystem.h"

void DsoAlgorithm::feedMonoImg(double ts, cv::Mat mono) {
    SlamTester::AlgorithmInterface::feedMonoImg(ts, mono);
/*        // Verify undistortion.
        for (auto &oi : output_interfaces) {
            oi->publishProcessImg(mono);
        }*/
    dso::ImageAndExposure image_for_dso(mono.cols, mono.rows, ts);
    float* IEdata = image_for_dso.image;
    auto w = mono.cols;
    if (mono.depth() == CV_16U) {
        typedef uint16_t Pixel;
        mono.forEach<Pixel>([w, IEdata](Pixel &p, const int *position) -> void {
            int idx = position[1] + position[0] * w;
            IEdata[idx] = p / 256.0; // Dso only accepts (0,256) intensity range.
        });
    } else if (mono.depth() == CV_8U) {
        typedef uint8_t Pixel;
        mono.forEach<Pixel>([w, IEdata](Pixel &p, const int *position) -> void {
            int idx = position[1] + position[0] * w;
            IEdata[idx] = p;
        });
    } else {
        LOG(ERROR) << "Unknown mono image data type.";
        return;
    }

    // static int i = 0;
    dso_algo->addActiveFrame(&image_for_dso, 0);

/*        // Verified that image_for_dso is packed correctly.
        cv::Mat debug_mat(mono.rows, mono.cols, CV_16U, cv::Scalar(255));
        typedef uint16_t Pixel;
        debug_mat.forEach<Pixel>([w, IEdata](Pixel &p, const int *position) -> void {
            int idx = position[1] + position[0] * w;
            p = static_cast<Pixel>(IEdata[idx]);
        });
        for (auto &oi : output_interfaces) {
            oi->publishProcessImg(debug_mat);
        }*/

}


void DsoAlgorithm::start() {
    Eigen::Matrix3f K;
    cv::cv2eigen(cv::Mat(input_interfaces[0]->inner_cam_k), K);
    // LOG(INFO) << "Eigen column major check K \n" << K;
    dso::setGlobalCalib(input_interfaces[0]->inner_w, input_interfaces[0]->inner_h, K);
    dso_algo = std::make_shared<dso::FullSystem>(this);
}
