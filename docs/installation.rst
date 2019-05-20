Installation
============

We assume that you have Ubuntu 16.04. Avoid using a virtual machine because this adds additional latency and might cause issues with the visualization tools.

.. warning::

    Using ubuntu in `Windows Subsystem for Linux (WSL) <https://docs.microsoft.com/en-us/windows/wsl/about>`_ is not supported since WSL does not have USB support and so Crazyradio will not work.
    You must install Ubuntu either directly on the computer or in a VM.

Simulation Only
---------------

You can install just the components required for the simulation by doing the following::

    $ sudo apt install git make gcc swig libpython-dev python-numpy python-yaml python-matplotlib
    $ git clone https://github.com/USC-ACTLab/crazyswarm.git
    $ cd crazyswarm
    $ ./buildSimOnly.sh

To test the installation, run one of the examples::

    $ cd ros_ws/src/crazyswarm/scripts
    $ python figure8_csv.py --sim

More details on the usage can be found in the :ref:`usage` section.

Simulation and Physical Robots
------------------------------

We assume that you have ROS Kinetic (desktop or desktop-full) installed (`instructions <http://wiki.ros.org/kinetic/Installation/Ubuntu>`_).

Install the dependencies and clone the repository::

    $ sudo apt install git swig libpython-dev python-numpy python-yaml python-matplotlib gcc-arm-none-eabi libpcl-dev libusb-1.0-0-dev sdcc ros-kinetic-vrpn-client-ros
    $ git clone https://github.com/USC-ACTLab/crazyswarm.git
    $ cd crazyswarm

You can now build everything by running our build script.::
    
    $ ./build.sh
