# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.10.2)

project(vins_mono)

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost 1.70 REQUIRED COMPONENTS filesystem)

add_subdirectory(camera_model)
add_subdirectory(feature_tracker)
add_subdirectory(vins_estimator)

add_library( # Sets the name of the library.
        ${PROJECT_NAME}
        # Sets the library as a shared library.
        SHARED
        # Provides a relative path to your source file(s).
        system.cpp
        VinsAlgorithm.cpp)

target_link_libraries(${PROJECT_NAME} # Specifies the target library.
        camera_model
        feature_tracker
        vins_estimator
        )

target_include_directories(${PROJECT_NAME} PUBLIC
)