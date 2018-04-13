ROOT=$PWD

# submodules
git submodule init
git submodule update

cd ros_ws/src/crazyflie_ros/
git submodule init
git submodule update
cd $ROOT

cd ros_ws/src/externalDependencies/libmotioncapture/
git submodule init
git submodule update
cd $ROOT

# build nrf firmware
cd crazyflie2-nrf-firmware
tools/build/download_deps
make
cd $ROOT

# build stm firmware
cd crazyflie-firmware
git submodule init
git submodule update
make
cd $ROOT

# build radio firmware
cd crazyradio-firmware/firmware
make CRPA=1
cd $ROOT

# build simulator firmware backend
cd ros_ws/src/crazyswarm/scripts/pycrazyswarm/cfsim
make
cd $ROOT

# ros
cd ros_ws
# -k: hack for dependency issues
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -k
catkin_make
cd $ROOT
