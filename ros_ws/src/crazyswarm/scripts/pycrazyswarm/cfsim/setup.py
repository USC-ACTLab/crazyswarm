"""Compiles the cffirmware C extension."""

from distutils.core import setup, Extension
import os

import numpy as np


fw_dir = "../../../../../../crazyflie-firmware"
include = [
    os.path.join(fw_dir, "src/modules/interface"),
    os.path.join(fw_dir, "src/hal/interface"),
    os.path.join(fw_dir, "src/utils/interface/lighthouse"),
    np.get_include(),
]

modules = [
    "collision_avoidance.c",
    "planner.c",
    "pptraj.c",
    "pptraj_compressed.c",
]
fw_sources = [os.path.join(fw_dir, "src/modules/src", mod) for mod in modules]

cffirmware = Extension(
    "_cffirmware",
    include_dirs=include,
    sources=fw_sources + ["cffirmware_wrap.c"],
    extra_compile_args=[
        "-O3",
        "-D__fp16=uint16_t",
    ],
)

setup(name="cffirmware", version="1.0", ext_modules=[cffirmware])
