Installation
============

We assume that you have Ubuntu 16.04. Avoid using a virtual machine because this adds additional latency and might cause issues with the visualization tools.

Simulation Only
---------------

You can install just the components required for the simulation by doing the following::

    $ sudo apt install git swig libpython-dev python-numpy python-yaml python-matplotlib
    $ git clone https://github.com/USC-ACTLab/crazyswarm.git
    $ cd crazyswarm
    $ ./buildSimOnly.sh

To test the installation, run one of the examples::

    $ cd ros_ws/src/crazyswarm/scripts
    $ python figure8_canned.py --sim

More details on the usage can be found in the :ref:`usage` section.

Simulation and Physical Robots
------------------------------

We assume that you have ROS Kinetic (desktop or desktop-full) installed (`instructions <http://wiki.ros.org/kinetic/Installation/Ubuntu>`_).

Install the dependencies and clone the repository::

    $ sudo apt install git swig libpython-dev python-numpy python-yaml python-matplotlib gcc-arm-none-eabi libpcl-dev libusb-1.0-0-dev sdcc
    $ git clone https://github.com/USC-ACTLab/crazyswarm.git
    $ cd crazyswarm

For legal reasons we are not allowed to include the VICON DataStream SDK in this repository, which is used to capture data from a VICON motion capture system. Please download the SDK (version 1.7.1) from http://www.vicon.com and place the following files in `ros_ws/src/crazyflie_ros/externalDependencies/libmotioncapture/externalDependencies/vicon_sdk` (that is, from the Linux64-boost-1.58.0 folder)::

    ├── include
    │   └── vicon_sdk
    │       └── DataStreamClient.h
    └── lib64
        ├── libboost_system-mt.so.1.58.0
        ├── libboost_thread-mt.so.1.58.0
        └── libViconDataStreamSDK_CPP.so

You can now build everything by running our build script.::
    
    $ ./build.sh
