cmake_minimum_required(VERSION 3.10)
project(vins_ros_wrapper)

# Build the talker and listener. Each one uses the following ROS packages,
# which we need to find_package() individually:
#   roscpp (the client library)
#   std_msgs (contains the std_msgs/String message type)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# find_package(glog REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(yaml-cpp REQUIRED)

# We've found them; now use their variables in the usual way to configure
# the compile and link steps.
# Note: we skip calling link_directories() because ROS packages follow the
# recommended CMake practice of returning absolute paths to libraries
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(${cv_bridge_INCLUDE_DIRS})

# add_executable(talker talker.cpp)
# target_link_libraries(talker ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES})

add_executable(ros_wrap_vins ros_wrap_vins.cpp)
target_link_libraries(ros_wrap_vins ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES} ${cv_bridge_LIBRARIES}
        # ${CMAKE_CURRENT_SOURCE_DIR}/../armpl-build/algorithms/vins_mono/libvins_mono.so
        # ${CMAKE_CURRENT_SOURCE_DIR}/../armpl-build/lib/libsocket_publisher.a
        # glog::glog
        # Eigen3::Eigen
        yaml-cpp
        vins_mono
        socket_publisher

        )

# (optional) Install the executables.
# install(TARGETS talker listener
#         DESTINATION bin)