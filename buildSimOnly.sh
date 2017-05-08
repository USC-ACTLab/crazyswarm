ROOT=$PWD

git submodule init
git submodule update

# get stm firmware
cd crazyflie-firmware
git submodule init
git submodule update
cd $ROOT

# build simulator firmware backend
cd ros_ws/src/crazyswarm/scripts/pycrazyswarm/cfsim
make
cd $ROOT

