cmake_minimum_required(VERSION 3.10.2)

project(rosbag)

find_package(glog REQUIRED)
find_package(OpenCV REQUIRED)

FILE(GLOB_RECURSE RosbagStorage_SOURCES
        src/RosbagStorage/*.c
        src/RosbagStorage/*.cpp
        )
add_library(${PROJECT_NAME} SHARED
        src/rostime/rate.cpp
        src/rostime/time.cpp
        src/rostime/duration.cpp
        src/cv_bridge_simple.cpp
        ${RosbagStorage_SOURCES}
        )
target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS}
        )
target_link_libraries(${PROJECT_NAME} PUBLIC
        ${OpenCV_LIBS}
        glog::glog
        )
