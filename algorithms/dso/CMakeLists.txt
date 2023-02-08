SET(PROJECT_NAME DSO)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
#set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# required libraries
find_package(SuiteSparse REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread)
find_package(glog REQUIRED)

# optional libraries
#find_package(LibZip QUIET)
find_package(Pangolin QUIET )
#message("==> Pangolin_LIBRARY : ${Pangolin_LIBRARY}")
find_package(OpenCV QUIET)

# flags
add_definitions("-DENABLE_SSE")
message(STATUS "SSE_FLAGS: " ${SSE_FLAGS})
#set(CMAKE_CXX_FLAGS
#   "${SSE_FLAGS} -O3 -g -std=c++0x -march=native"
##   "${SSE_FLAGS} -O3 -g -std=c++0x -fno-omit-frame-pointer"
#)

# Sources files
set(dso_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystem.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemOptimize.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemOptPoint.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemDebugStuff.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemMarginalize.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/Residuals.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/CoarseTracker.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/CoarseInitializer.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/ImmaturePoint.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/HessianBlocks.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/PixelSelector2.cpp
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/EnergyFunctional.cpp
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/AccumulatedTopHessian.cpp
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/AccumulatedSCHessian.cpp
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/EnergyFunctionalStructs.cpp
  ${PROJECT_SOURCE_DIR}/src/util/settings.cpp
  ${PROJECT_SOURCE_DIR}/src/util/Undistort.cpp
  ${PROJECT_SOURCE_DIR}/src/util/globalCalib.cpp

        ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
        ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageRW_OpenCV.cpp
        DsoAlgorithm.cpp)


# compile main library.
add_library(dso SHARED ${dso_SOURCE_FILES})
target_link_libraries(dso Boost::system ${CSPARSE_LIBRARY} Boost::thread glog::glog Eigen3::Eigen ${OpenCV_LIBS})
#set_property( TARGET dso APPEND_STRING PROPERTY COMPILE_FLAGS -Wall )
target_include_directories(dso PUBLIC
		${PROJECT_SOURCE_DIR}/src
		${PROJECT_SOURCE_DIR}/thirdparty/Sophus
		${PROJECT_SOURCE_DIR}/thirdparty/sse2neon
		#  ${EIGEN3_INCLUDE_DIR}
		${OpenCV_INCLUDE_DIRS}
		${CSPARSE_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR}
		)

#message("==> CMAKE_SYSTEM_NAME : ${CMAKE_SYSTEM_NAME}")
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin") # OSX
#    set(BOOST_THREAD_LIBRARY boost_thread-mt)
	add_definitions(-D_OSX_)
	add_definitions(-DHAVE_GLEW)
else()
#    set(BOOST_THREAD_LIBRARY boost_thread)
endif()



