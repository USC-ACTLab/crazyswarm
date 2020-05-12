"""Compiles the cffirmware C extension."""

from distutils.core import setup, Extension
import os

fw_dir = "../../../../../../crazyflie-firmware"
include = [
    os.path.join(fw_dir, "src/modules/interface"),
    os.path.join(fw_dir, "src/hal/interface"),
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
    extra_compile_args=["-O3"],
)

setup(name="cffirmware", version="1.0", ext_modules=[cffirmware])
