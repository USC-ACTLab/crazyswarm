# crazyswarm
How to fly a large swarm of Crazyflies

## Description
This repository should be all you need to set up a development environment for the USC Crazyflie swarm.
It mostly consists of links to other submodules, tied together with a build script.
The submodules are:

repo | description
---- | -----------
**crazyflie-clients-python**  | The official Crazyflie GUI client. Not customized. Useful for manual flight and sanity checks. |
**crazyflie-firmware**        | Our fork of the software that runs onboard the Crazyflie on the main CPU. Extensively modified with new controller, EKF, spline trajectories, etc.
**crazyflie-lib-python**      | A dependency of **crazyflie-clients-python**.
**crazyflie2-nrf-firmware**   | Our fork of the onboard software that controls the Crazyflie's radio. Modified to support broadcast communication to swarms.
**crazyradio-firmware**       | (Public) fork of the Crazyradio firmware. Modified to support broadcasts.
**ros_ws/src/crazyflie_ros**  | The main ROS stack dealing with sending commands and receiving telemetry from the Crazyflie in ROS.
**ros_ws/src/object_tracker** | Our custom object tracker that, unlike Vicon's object tracker, doesn't require unique marker configurations per vehicle.
**ros_ws/src/vicon_ros**      | A ROS interface to the normal Vicon object tracking (that requires unique marker configurations).

Git submodules behave in a somewhat counterintuitive way.
They point to a *specific commit* in the submodule's commit tree.
They don't point to a named branch head like you might expect.
Therefore, if you modify one of the submodules listed above, you must also modify this repository
via `git submodule update` so it points to your new commit.

## Prerequisites
- Install ROS
- Install GNU ARM toolchain following [these instructions](https://github.com/bitcraze/crazyflie-firmware#install-a-toolchain)
- ??? (create an issue/PR if there are some we missed)

## Setup
```
git clone https://github.com/USC-ACTLab/crazyswarm2.git
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
./JLinkExe -if swd -device STM32F405RG -speed 4000
```

and type `connect`. In a second terminal:

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
