#!/bin/bash
modulesFolder=/home/xbpeng/scratch/modules

CPLUS_INCLUDE_PATH=$modulesFolder/bzip2/include/:$modulesFolder/boost_1_55_0:$modulesFolder/protobuf-2.6.1/src:$modulesFolder/gflags-2.1.2/build/include:$modulesFolder/OpenBLAS-0.2.15:$modulesFolder/hdf5-1.8.16/src:$modulesFolder/opencv-2.4.11/include
CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:$modulesFolder/opencv-2.4.11/modules/highgui/include
CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:$modulesFolder/opencv-2.4.11/modules/core/include
CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:$modulesFolder/opencv-2.4.11/modules/imgproc/include
CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:$modulesFolder/hdf5-1.8.16/hl/src
CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:$modulesFolder/lmdb/libraries/liblmdb
CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:$modulesFolder/leveldb/include
CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:$modulesFolder/glog-0.3.4/src
export CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:$modulesFolder/gcc/include
export CPLUS_INCLUDE_PATH

LIBRARY_PATH=/scratch/users/xbpeng/packages/bzip2/lib/:$modulesFolder/boost_1_55_0/stage/lib:$modulesFolder/gflags-2.1.2/build/lib:$modulesFolder/OpenBLAS-0.2.15:$modulesFolder/opencv-2.4.11/release/lib:$modulesFolder/ncurses-6.0/lib:$modulesFolder/glog-0.3.4/.libs:$modulesFolder/protobuf-2.6.1/src/.libs:$modulesFolder/hdf5-1.8.16/hl/src/.libs:$modulesFolder/hdf5-1.8.16/src/.libs:$modulesFolder/leveldb:$modulesFolder/snappy-1.1.3/.libs:$modulesFolder/lmdb/libraries/liblmdb
export LIBRARY_PATH

LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$modulesFolder/Python-2.7.3:$modulesFolder/boost_1_55_0/stage/lib:$modulesFolder/gcc/lib:/scratch/users/xbpeng/packages/bzip2/lib/:$modulesFolder/boost_1_55_0/stage/lib:$modulesFolder/gflags-2.1.2/build/lib:$modulesFolder/OpenBLAS-0.2.15:$modulesFolder/opencv-2.4.11/release/lib:$modulesFolder/ncurses-6.0/lib:$modulesFolder/glog-0.3.4/.libs:$modulesFolder/protobuf-2.6.1/src/.libs:$modulesFolder/hdf5-1.8.16/hl/src/.libs:$modulesFolder/hdf5-1.8.16/src/.libs:$modulesFolder/leveldb:$modulesFolder/snappy-1.1.3/.libs:$modulesFolder/lmdb/libraries/liblmdb
export LD_LIBRARY_PATH

PATH=$PATH:$modulesFolder/protobuf-2.6.1/src
PATH=$PATH:$modulesFolder/premake-4.4-beta5/bin/release
PATH=$PATH:$modulesFolder/cmake-3.4.1-Linux-x86_64/bin
PATH=$PATH:$modulesFolder/swig-3.0.10:$modulesFolder/gcc
export PATH

PYTHONPATH=$HOME/local/lib/python2.7/site-packages
export PYTHONPATH

# for numpy and scipy
export BLAS=$modulesFolder/OpenBLAS-0.2.15/libcblas.so
export ATLAS=$modulesFolder/OpenBLAS-0.2.15/libatlas.so
export LAPACK_SRC=$modulesFolder/OpenBLAS-0.2.15/lapack/
export LAPACK=/usr/lib64/atlas/liblapack.so.3

export SWIG_LIB=$modulesFolder/swig-3.0.10/Lib/

