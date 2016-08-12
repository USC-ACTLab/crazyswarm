#!/usr/bin/env python3

import yaml
import subprocess
import argparse
import os

def main():
    parser = argparse.ArgumentParser(description='Flash firmware.')
    parser.add_argument('-stm32', action="store_true")
    parser.add_argument('-nrf51', action="store_true")
    args = parser.parse_args()

    if not (args.nrf51 or args.stm32):
        print("-stm32, -nrf51, or both?")
        return

    os.chdir(os.path.dirname(os.path.realpath(__file__)))

    with open("../ros_ws/src/crazyswarm/launch/crazyflies.yaml", 'r') as ymlfile:
        cfg = yaml.load(ymlfile)

    for crazyflie in cfg["crazyflies"]:
        id = "{0:02X}".format(crazyflie["id"])
        uri = "radio://0/{}/2M/E7E7E7E7{}".format(crazyflie["channel"], id)
        if args.nrf51:
            print("Flash NRF51 FW to {}".format(uri))
            subprocess.call(["../crazyflie-clients-python/bin/cfloader -w " + uri + " flash ../crazyflie2-nrf-firmware/cf2_nrf.bin nrf51-fw"], shell=True)
        if args.stm32:
            print("Flash STM32 FW to {}".format(uri))
            subprocess.call(["../crazyflie-clients-python/bin/cfloader -w " + uri + " flash ../crazyflie-firmware/cf2.bin stm32-fw"], shell=True)


if __name__ == "__main__":
    main()
