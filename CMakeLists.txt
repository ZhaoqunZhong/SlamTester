cmake_minimum_required(VERSION 3.23)
project(SlamTester)

# make stop at first error
add_definitions(-Wfatal-errors)
# Disable warning in make process
add_definitions(-w)

include(/usr/local/lib/cmake/colorMessage.cmake)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -fPIC")

find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(glog REQUIRED)
find_package(yaml-cpp REQUIRED)
#message("==> OpenCV_INCLUDE_DIRS : ${OpenCV_INCLUDE_DIRS}")
#message("==> OpenCV_LIBS: ${OpenCV_LIBS}")

add_subdirectory(socket_publisher)
add_subdirectory(rosbag)
add_subdirectory(algorithms/vins_mono)
add_subdirectory(algorithms/dso)

add_executable(${PROJECT_NAME}
        main.cpp
        PangolinViewer.cpp
        SlamInterface.cpp
        DataSets.cpp
        alignment/AlignUtils.cpp
        alignment/AlignTrajectory.cpp
        algorithms/Pangolin_Algo_Example.cpp
        backward.cpp
        )

target_link_libraries(${PROJECT_NAME} PUBLIC
        pango_display
        Eigen3::Eigen
        rosbag
        ${OpenCV_LIBS}
        glog::glog
        yaml-cpp
        socket_publisher
        vins_mono
        dso
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        ${OpenCV_INCLUDE_DIRS}
        )

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin") # MacOS
    target_include_directories(${PROJECT_NAME} PUBLIC
            /opt/homebrew/include # for GL/glew.h
            )
endif ()