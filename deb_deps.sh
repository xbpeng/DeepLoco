#! /bin/bash
#
# Install the system dependencies required to build TerrainRL
#
sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler -y
sudo apt-get install --no-install-recommends libboost-all-dev -y
sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev -y
sudo apt-get install libatlas-base-dev -y
sudo apt-get install gcc-4.9-multilib g++-4.9-multilib -y
sudo apt-get install libf2c2-dev -y
sudo apt-get install libglew-dev -y
## This is not necessary
#  sudo apt-get install cuda
