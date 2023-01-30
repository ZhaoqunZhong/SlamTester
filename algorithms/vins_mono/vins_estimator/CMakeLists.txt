cmake_minimum_required(VERSION 3.10.2)
project(vins_estimator)

add_library(${PROJECT_NAME} SHARED
#    src/estimator_node.cpp
    src/parameters.cpp
    src/estimator.cpp
    src/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projection_factor.cpp
    src/factor/projection_td_factor.cpp
    src/factor/marginalization_factor.cpp
    src/utility/utility.cpp
#    src/utility/visualization.cpp
#    src/utility/CameraPoseVisualization.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    )


target_link_libraries(${PROJECT_NAME} PUBLIC
        feature_tracker
        ${OpenCV_LIBS}
        Eigen3::Eigen
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        ${OpenCV_INCLUDE_DIRS})
