"""Compiles the cffirmware C extension."""

from distutils.core import setup, Extension
import os

fw_dir = "../../../../../../crazyflie-firmware"
fw_include = os.path.join(fw_dir, "src/modules/interface")

modules = [
    "planner.c",
    "pptraj.c",
    "pptraj_compressed.c",
]
fw_sources = [os.path.join(fw_dir, "src/modules/src", mod) for mod in modules]

cffirmware = Extension(
    "_cffirmware",
    include_dirs=[fw_include],
    sources=fw_sources + ["cffirmware_wrap.c"],
    extra_compile_args=["-O3"],
)

setup(name="cffirmware", version="1.0", ext_modules=[cffirmware])
