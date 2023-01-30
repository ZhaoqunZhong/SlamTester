cmake_minimum_required(VERSION 3.10.2)
project(camera_model)


add_library(${PROJECT_NAME} SHARED
#    src/chessboard/Chessboard.cc
#    src/calib/CameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc)

#target_link_libraries(Calibration ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC
        Eigen3::Eigen
        ${CERES_LIBRARIES}
        ${OpenCV_LIBS}
        Boost::filesystem
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        )
