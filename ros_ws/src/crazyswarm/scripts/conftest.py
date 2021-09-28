import subprocess

import pytest


HAVE_ROS = subprocess.run("type roslaunch", shell=True).returncode == 0


def pytest_runtest_setup(item):
    markers = [mark.name for mark in item.iter_markers()]
    if "ros" in markers and not HAVE_ROS:
        pytest.skip("ROS is not installed.")
