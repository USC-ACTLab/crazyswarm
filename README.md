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

## Crazyradio

```
cd crazyradio-firmware/firmware
make CRPA=1
python ../usbtools/launchBootloader.py
python ../usbtools/nrfbootload.py flash bin/cradio.bin

```

Unplug and replug radio.

## Kalman Filter
Initial test - using [suhetao's EKF](https://github.com/suhetao/stm32f4_mpu9250)

- predict + update takes 1606 uSec (stddev = 63 uSec)
- so 500hz predict + 100hz update is probably possible!
- and backtracking for time sync should be OK
- 16 states: quat, position, velocity, gyro bias, accelerometer bias
- using special ARM math library

### Data Collection

Use https://github.com/whoenig/crazyflie-firmware/tree/cs_datacollection as firmware.
To collect data using SEGGER Real-Time Transfers (RRT), run

```
./JLinkExe -if swd -device STM32F405RG -speed 2000
```

and type `connect`. In a second terminal, from the scripts folder, execute `python3 RTT.py`.

## Radio Bandwidth

## Vicon
