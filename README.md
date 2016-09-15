# crazyswarm
A Large Nano-Quadcopter Swarm.

## Description
This repository should be all you need to set up a development environment for the USC Crazyflie swarm.
A high-level overview is given in our scientific papers "Crazyswarm: A Large Nano-Quadcopter Swarm", by James A. Preiss
, Wolfgang Hönig, Gaurav S. Sukhatme, and Nora Ayanian, published as extended abstract at IROS 2016 and submitted for review to ICRA 2017.

It mostly consists of links to other submodules, tied together with a build script.
The submodules are:

repo | description
---- | -----------
**crazyflie-clients-python**  | The official Crazyflie GUI client. Not customized. Useful for manual flight and sanity checks. |
**crazyflie-firmware**        | Our fork of the software that runs onboard the Crazyflie on the main CPU. Extensively modified with new controller, EKF, spline trajectories, etc.
**crazyflie-lib-python**      | A dependency of **crazyflie-clients-python**. Slightly modified for improved flashing.
**crazyflie2-nrf-firmware**   | Our fork of the onboard software that controls the Crazyflie's radio. Modified to support broadcast communication to swarms.
**crazyradio-firmware**       | (Public) fork of the Crazyradio firmware. Modified to support broadcasts.
**ros_ws/src/crazyflie_ros**  | The main ROS stack dealing with sending commands and receiving telemetry from the Crazyflie in ROS. Modified for low latency and swarm support.

Git submodules behave in a somewhat counterintuitive way.
They point to a *specific commit* in the submodule's commit tree.
They don't point to a named branch head like you might expect.
Therefore, if you modify one of the submodules listed above, you must also modify this repository
via `git submodule update` so it points to your new commit.

## Prerequisites
- Install ROS
- Install GNU ARM toolchain following [these instructions](https://github.com/bitcraze/crazyflie-firmware#install-a-toolchain)
- For legal reasons we are not allowed to include the VICON DataStream SDK in this repository, which is used to capture data from a VICON motion capture system.
  Please download the SDK from http://www.vicon.com and place the following files in `ros_ws/src/crazyflie_ros/externalDependencies/vicon_sdk`
```
├── include
│   └── vicon_sdk
│       └── Client.h
└── lib64
    ├── libboost_locale-mt.so.1.53.0
    ├── libboost_system-mt.so.1.53.0
    ├── libboost_thread-mt.so.1.53.0
    ├── libboost_unit_test_framework-mt.so.1.53.0
    ├── libDebugServices.so
    └── libViconDataStreamSDK_CPP.so
```
- ??? (create an issue/PR if there are some we missed)

## Setup
```
git clone https://github.com/USC-ACTLab/crazyswarm.git
./build.sh
```

## Firmware (NRF)

```
cd crazyflie2-nrf-firmware
tools/build/download_deps
make
make cload
```

## Crazyradio

```
cd crazyradio-firmware/firmware
make CRPA=1
python ../usbtools/launchBootloader.py
python ../usbtools/nrfbootload.py flash bin/cradio.bin
```

Unplug and replug radio.

## Debugging

### Firmware

Use SEGGER Real-Time Transfers (RRT):

```
./JLinkExe -if swd -device STM32F405RG -speed 4000 -autoconnect 1
```

In a second terminal:

```
./JLinkRTTClient
```

## Simulation

### Setup

- Clone this repository
- Clone the firmware into crazyflie-firmware
- ```cd /ros_ws/src/crazyswarm/scripts/pycrazyswarm/cfsim```
- ```make``` (this builds the simulator components which use the firmware)

If you want to use the vispy backend:
- Follow instructions at https://github.com/vispy/vispy

### Usage (Matplotlib)

- Enable a subset of Crazyflies by editing ```ros_ws/src/crazyswarm/launch/crazyflies.yaml```
- ```cd ros_ws/src/crazyswarm/scripts```
- ```python figure8_canned.py --sim```

### Usage (Vispy)

- ```cd ros_ws/src/crazyswarm/scripts```
- ```python figure8_canned.py --sim --vis vispy --dt 0.01```

## Usage

- The Crazyflies (initial position and channels) are defined in `ros_ws/src/crazyswarm/launch/crazyflies.yaml`. You can use the GUI in `ros_ws/src/crazyswarm/scripts/chooser.py` to enable or disable individual Crazyflies and execute commands such as powering them on- or off.
- Use `roslaunch crazyswarm hover_swarm.launch` to connect to the Crazyflies and show the current tracking state in `rviz`.
- Run one of the scripts in `ros_ws/src/crazyswarm/scripts` to execute a motion.
