# crazyswarm
How to fly a large swarm of Crazyflies

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

### Data Collection

Use https://github.com/whoenig/crazyflie-firmware/tree/cs_datacollection as firmware.
To collect data using SEGGER Real-Time Transfers (RRT), run

```
./JLinkExe -if swd -device STM32F405RG -speed 2000
```

and type `connect`. In a second terminal, from the scripts folder, execute `python3 RTT.py`.

## Radio Bandwidth

## Vicon
