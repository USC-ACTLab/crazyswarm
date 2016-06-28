# build nrf firmware
cd crazyflie2-nrf-firmware
tools/build/download_deps
make
cd ..

# build stm firmware
cd crazyflie-firmware
make
cd ..

# ros
cd ros_ws
catkin_make
cd ..
