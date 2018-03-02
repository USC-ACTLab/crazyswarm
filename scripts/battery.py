#!/usr/bin/env python3

import yaml
import subprocess
import os

os.chdir(os.path.dirname(os.path.realpath(__file__)))

with open("../ros_ws/src/crazyswarm/launch/crazyflies.yaml", 'r') as ymlfile:
    cfg = yaml.load(ymlfile)
with open("../ros_ws/src/crazyswarm/launch/crazyflieTypes.yaml", 'r') as ymlfile:
    cfTypes = yaml.load(ymlfile)
    cfTypes = cfTypes["crazyflieTypes"]

for crazyflie in cfg["crazyflies"]:
    id = "{0:02X}".format(crazyflie["id"])
    uri = "radio://0/{}/2M/E7E7E7E7{}".format(crazyflie["channel"], id)
    print("{}: ".format(crazyflie["id"]), end="", flush=True)
    cfType = crazyflie["type"]
    bigQuad = cfTypes[cfType]["bigQuad"]

    if not bigQuad:
        subprocess.call(["rosrun crazyflie_tools battery --uri " + uri], shell=True)
    else:
        subprocess.call(["rosrun crazyflie_tools battery --uri " + uri + " --external 1"], shell=True)
