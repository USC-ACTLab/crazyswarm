#!/usr/bin/env python3

"""
Helper script to do something to multiple crazyflies on the same channel 
simultaneously. Specifically, running ``crazyflie`` ROS package scripts
such as *battery*, *reboot*, and *sysoff* for multiple crazyflies instead
of running them manually for different UID's. 

NOTE: this serves a similar pupose to ``chooser.py`` but can be ran from 
the command line.
"""

import argparse
from pathlib import Path
import subprocess

from ruamel.yaml import YAML


SUBPROC_TIMEOUT = 1
VALID_SUBCMDS = {
   "battery", 
   "reboot", 
   "sysoff", 
   "version",
   "listLogVariables",
   "listParams", 
   "listMemories"
}


def main():
    parser = argparse.ArgumentParser(
        description="Wrapper for 'ros2 run crazyflie ...' to multiplex '...' across crazyflies"
    )
    parser.add_argument("subcommand", help="Subcommand of crazyflie to be ran",
            choices=VALID_SUBCMDS)
    sub_parser = parser.add_subparsers()
    man_parser = sub_parser.add_parser("manual", help="Manually specify channels and UID's")
    man_parser.add_argument("-c", "--channel", default=80, 
            help="Channel of this Crazyflie(s)")
    man_parser.add_argument("-u", "--uids", nargs="+", required=True,
            help="Last 1-2 characters of UID's of each of the crazyflies in hex.")
    
    yaml_parser = sub_parser.add_parser("yaml", help="Load radio addressed via yaml file")
    yaml_parser.add_argument("-C", "--configpath",
		default=Path(__file__).parent.parent.resolve() / "config",
		help="Path to the configuration *.yaml files")

    args = parser.parse_args()

    args.subcommand = [args.subcommand]
    if args.subcommand[0] == "sysoff":
        args.subcommand = ["reboot", "--mode=sysoff"]

    # Determine which URI's to mess with.
    if getattr(args, 'configpath', None):
        uris = _read_yaml_uris(Path(args.configpath).resolve())
    else:
        if not all(len(uid) <= 2 for uid in args.uids):
            raise ValueError("UID must be a 1-2 element string")
        uris = [f"radio://0/{args.channel}/2M/E7E7E7E7{uid.zfill(2)}" for uid in args.uids]
    
    # Run for all URI's determined.
    for uri in uris:
        cmd = [
            "ros2", 
            "run", 
            "crazyflie", 
            *args.subcommand, 
            f"--uri={uri}",
        ]
        print(f"{' '.join(cmd)}")
        subprocess.run(cmd, timeout=SUBPROC_TIMEOUT)


def _read_yaml_uris(configpath):
    if not configpath.is_dir():
       raise ValueError(f"configpath {configpath} input does not exist.")
    yamlpath = configpath / "crazyflies.yaml"
    if not yamlpath.is_file():
        raise FileNotFoundError(f"{yamlpath} not found in {configpath}.")
        
    yaml = YAML()
    cfg = yaml.load(yamlpath)
    
    return [val['uri'] for val in cfg['robots'].values() if val['enabled']]


if __name__ == "__main__":
    main()
