if [ -d "ubuntu_build" ]; then
    cd ubuntu_build
    rm -r *
else
    mkdir ubuntu_build
    cd ubuntu_build
fi

cmake \
	-DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DEigen3_DIR=/home/zhaoqun/Documents/libs_local_install/share/eigen3/cmake \
    -DOpenCV_DIR=/home/zhaoqun/Documents/open_vslam/dependencies/share/OpenCV \
    -Dglog_DIR=/home/zhaoqun/Documents/libs_local_install/lib/cmake/glog \
    -Dgflags_DIR=/home/zhaoqun/Documents/libs_local_install/lib/cmake/gflags \
    -Dyaml-cpp_DIR=/home/zhaoqun/Documents/libs_local_install/lib/cmake/yaml-cpp \
    -Dsioclient_DIR=/home/zhaoqun/Documents/libs_local_install/lib/cmake/sioclient \
    -Dprotobuf_DIR=/home/zhaoqun/Documents/libs_local_install/lib/cmake/protobuf \
    -DCeres_DIR=/home/zhaoqun/Documents/libs_local_install/lib/cmake/Ceres \
    -DSuiteSparse_DIR=/home/zhaoqun/Documents/libs_local_install/lib/cmake/suitesparse-5.4.0 ..

make -j4