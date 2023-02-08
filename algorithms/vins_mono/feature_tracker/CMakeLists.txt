cmake_minimum_required(VERSION 3.10.2)
project(feature_tracker)

add_library(feature_tracker STATIC
#    src/feature_tracker_node.cpp
        src/parameters.cpp
    src/feature_tracker.cpp
    )

target_link_libraries(feature_tracker PUBLIC
        camera_model
        ${OpenCV_LIBS}
        Eigen3::Eigen
        Boost::filesystem)

target_include_directories(${PROJECT_NAME} PUBLIC
        ${OpenCV_INCLUDE_DIRS}
        ../camera_model/include)