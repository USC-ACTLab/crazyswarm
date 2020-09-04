Installation
============

For real hardware operation, we assume that you have **Ubuntu 16.04 with ROS Kinetic** or **Ubuntu 18.04 with ROS Melodic** or **Ubuntu 20.04 with ROS Noetic** .
Avoid using a virtual machine because this adds additional latency and might cause issues with the visualization tools.

For simulation-only operation, **MacOS** is also supported.

.. warning::

    Using ubuntu in `Windows Subsystem for Linux (WSL) <https://docs.microsoft.com/en-us/windows/wsl/about>`_ is not supported since WSL does not have USB support and so Crazyradio will not work.
    You must install Ubuntu either directly on the computer or in a VM.


Simulation Only
---------------

It is possible to write/debug ``pycrazyswarm`` scripts and selected firmware modules
on a machine that does not have ROS or the ARM cross-compilation toolchain installed.
You can install just the components required for the simulation by doing the following:

----

Linux or MacOS with Anaconda
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

First, install `Anaconda Python 2.7 / 3.7 version <https://www.anaconda.com/distribution>`_.
We have tested on version ``2019.10``.
Next, clone the Crazyswsarm repo and build the Anaconda environment with the specific python version number::

    $ git clone https://github.com/USC-ACTLab/crazyswarm.git
    $ cd crazyswarm
    $ conda env create python=[DESIRED PYTHON VERSION] -f conda_env.yaml

Activate the Anaconda environment. Set the CSW_PYTHON variable to either python2 or python3 and run the build script::

    $ conda activate crazyswarm
    $ CSW_PYTHON=[DESIRED PYTHON COMMAND] ./buildSimOnly.sh

----

Linux without Anaconda
~~~~~~~~~~~~~~~~~~~~~~

Install the dependencies. Set the CSW_PYTHON variable to either python2 or python3 and run the build script::
    $ export CSW_PYTHON=python3
    $ sudo apt install git make gcc swig lib${CSW_PYTHON}-dev ${CSW_PYTHON}-numpy ${CSW_PYTHON}-yaml ${CSW_PYTHON}-matplotlib
    $ git clone https://github.com/USC-ACTLab/crazyswarm.git
    $ cd crazyswarm
    $ ./buildSimOnly.sh

----

In either case, to test the installation, run one of the examples::

    $ cd ros_ws/src/crazyswarm/scripts
    $ python figure8_csv.py --sim

More details on the usage can be found in the :ref:`Usage` section.


Simulation and Physical Robots
------------------------------

For real hardware operation, we assume that you have **Ubuntu 16.04 with ROS Kinetic** or **Ubuntu 18.04 with ROS Melodic** or **Ubuntu 20.04 with ROS Noetic**.

- Ubuntu 16.04, ROS Kinetic, Python2
    with ROS Kinetic(desktop or desktop-full) installed (`instructions <http://wiki.ros.org/kinetic/Installation/Ubuntu>`_).
- Ubuntu 18.04, ROS Melodic, Python2
    with ROS Melodic(desktop or desktop-full) installed (`instructions <http://wiki.ros.org/melodic/Installation/Ubuntu>`_).
- Ubuntu 20.04, ROS Noetic, Python3
    with ROS Noetic(desktop or desktop-full) installed (`instructions <http://wiki.ros.org/noetic/Installation/Ubuntu>`_).

Install the dependencies. Set the CSW_PYTHON variable to either python2 or python3 and clone the repository::
    $ export CSW_PYTHON=python3
    $ sudo apt install git swig lib${CSW_PYTHON}-dev ${CSW_PYTHON}-numpy ${CSW_PYTHON}-yaml ${CSW_PYTHON}-matplotlib gcc-arm-embedded libpcl-dev libusb-1.0-0-dev sdcc ros-[ROS version]-vrpn
    $ git clone https://github.com/USC-ACTLab/crazyswarm.git
    $ cd crazyswarm

You can now build everything by running our build script.::
    
    $ ./build.sh
