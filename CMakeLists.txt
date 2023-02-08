cmake_minimum_required(VERSION 3.10)
project(SlamTester)

# find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(glog REQUIRED)
find_package(yaml-cpp REQUIRED)
#message("==> OpenCV_INCLUDE_DIRS : ${OpenCV_INCLUDE_DIRS}")
#message("==> OpenCV_LIBS: ${OpenCV_LIBS}")

add_subdirectory(socket_publisher)
add_subdirectory(rosbag)
add_subdirectory(algorithms/vins_mono)
add_subdirectory(algorithms/dso)

# set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
# set(BUILD_SHARED_LIBS OFF)
# set(CMAKE_EXE_LINKER_FLAGS "-static")

add_subdirectory(ros_wrap)

add_executable(${PROJECT_NAME} 
        main.cpp
        # PangolinViewer.cpp
        SlamInterface.cpp
        DataSets.cpp
        alignment/AlignUtils.cpp
        alignment/AlignTrajectory.cpp
        # algorithms/Pangolin_Algo_Example.cpp
        )

target_link_libraries(${PROJECT_NAME} PUBLIC
        # pango_display
        Eigen3::Eigen
        custom_rosbag
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


