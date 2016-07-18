ROOT=$PWD

# submodules
git submodule init
git submodule update

cd ros_ws/src/vicon_ros/
git submodule init
git submodule update
cd $ROOT

cd ros_ws/src/object_tracker/
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
make
cd $ROOT

# ros
cd ros_ws
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo
cd $ROOT
