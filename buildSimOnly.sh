# Exit immediately if a command exits with a non-zero status.
set -e

ROOT=$PWD

git submodule init
git submodule update crazyflie-firmware

# build simulator firmware backend
cd ros_ws/src/crazyswarm/scripts/pycrazyswarm/cfsim
make
cd $ROOT

