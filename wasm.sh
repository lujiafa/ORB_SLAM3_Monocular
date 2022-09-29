#!/bin/bash

PREFIX_PATH=/home/jon/workspace2/ORB_SLAM3_Monocular

mkdir -p $PREFIX_PATH/output/res

# cp -rf $PREFIX_PATH/Vocabulary/ORBvoc.bin $PREFIX_PATH/output/res
# cp -rf $PREFIX_PATH/Examples/Monocular/TUM1.yaml $PREFIX_PATH/output/res
$PREFIX_PATH/output

emcmake /home/jon/soft/cmake-3.22.5-linux-x86_64/bin/cmake \
-DBUILD_SHARED_LIBS=OFF \
-DBUILD_TESTS=OFF \
$PREFIX_PATH

emmake make -j6
