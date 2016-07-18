#!/usr/bin/env python3

import yaml
import subprocess
import argparse

parser = argparse.ArgumentParser(description='Flash firmware.')
parser.add_argument('stm32', nargs='?')
parser.add_argument('nrf51', nargs='?')
args = parser.parse_args()

with open("../ros_ws/src/crazyswarm/launch/crazyflies.yaml", 'r') as ymlfile:
    cfg = yaml.load(ymlfile)

for crazyflie in cfg["crazyflies"]:
    uri = "radio://0/100/2M/E7E7E7E7" + crazyflie["id"]
    if args.nrf51 is not None:
        subprocess.call(["../crazyflie-clients-python/bin/cfloader -w " + uri + " flash ../crazyflie2-nrf-firmware/cf2_nrf.bin nrf51-fw"], shell=True)
    if args.stm32 is not None:
        subprocess.call(["../crazyflie-clients-python/bin/cfloader -w " + uri + " flash ../crazyflie-firmware/cf2.bin stm32-fw"], shell=True)
