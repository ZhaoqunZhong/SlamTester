if [ -d "build" ]; then
    cd build
    rm -r *
else
    mkdir build
    cd build
fi

cmake \
	-DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_CXX_STANDARD=14 \
    ..

make -j6