if [ -d "build" ]; then
    cd build
    rm -r *
else
    mkdir build
    cd build
fi

cmake \
	-DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_CXX_STANDARD=17 \
    ..

make -j6