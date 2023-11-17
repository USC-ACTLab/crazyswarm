.. _installation:

Installation
============

Crazyswarm2 runs on **Ubuntu Linux** in one of the following configurations:

====== ======== ============
Ubuntu Python   ROS 2
------ -------- ------------
22.04  3.10     Humble, Iron
====== ======== ============

.. warning::
   The `Windows Subsystem for Linux (WSL) <https://docs.microsoft.com/en-us/windows/wsl/about>`_ is experimentally supported but you'll have to use `usbipd-win <https://github.com/dorssel/usbipd-win/>`_.
   This program will link the crazyradio directly with WS, but beware of bugs. Check out their `WSL connect guide <https://github.com/dorssel/usbipd-win/wiki/WSL-support/>`_.

.. warning::
   Avoid using a virtual machine if possible: they add additional latency and might cause issues with the visualization tools.

First Installation
------------------

1. If needed, install ROS 2 using the instructions at https://docs.ros.org/en/galactic/Installation.html or https://docs.ros.org/en/humble/Installation.html.

2. Install dependencies

    .. code-block:: bash

        sudo apt install libboost-program-options-dev libusb-1.0-0-dev
        pip3 install rowan

    If you are planning to use the CFlib backend, do:

    .. code-block:: bash
        
        pip3 install cflib transforms3d
        sudo apt-get install ros-*DISTRO*-tf-transformations

3. Set up your ROS 2 workspace

    .. code-block:: bash

        mkdir -p ros2_ws/src
        cd ros2_ws/src
        git clone https://github.com/IMRCLab/crazyswarm2 --recursive
        git clone --branch ros2 --recursive https://github.com/IMRCLab/motion_capture_tracking.git

4. Build your ROS 2 workspace

    .. code-block:: bash

        cd ../
        colcon build --symlink-install

    .. note::
       symlink-install allows you to edit Python and config files without running `colcon build` every time.

5. Set up Crazyradio

   For the crazyradio, you need to setup usb rules in order to communicate with the Crazyflie. Find the instructions for that here `in Bitcraze's USB permission guide for Linux <https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/>`_.

   You will also need to update the crazyradio firmware to the latest development branch to be able to use all features. For Crazyradio PA (1), `follow these instructions <https://www.bitcraze.io/documentation/repository/crazyradio-firmware/master/building/building_flashing/>`_. For Crazyradio 2, follow `these instuctions to build the firmware <https://www.bitcraze.io/documentation/repository/crazyradio-firmware/master/building/building_flashing/>`_ and `these instuctions to flash it <https://www.bitcraze.io/documentation/repository/crazyradio2-firmware/main/building-and-flashing/flash//>`_.

6. Update the crazyflies

   If this is the first time handling crazyflies it is always good to start with `Bitcraze's getting started guide  <https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/>`_.

   You can update each crazyflie firmware to the latest release via `these instructions of the Bitcraze Crazyflie client <https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/userguide_client/#firmware-upgrade>`_ .

   While you are at it, make sure that each crazyflie have an unique radio address which you can change in `the client via these instructions <https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/userguide_client/#firmware-configuration>`_ .

7. Set up software-in-the-loop simulation (optional)

    This currently requires cloning the Crazyflie firmware and building the Python bindings manually. In a separate folder (not part of your ROS 2 workspace!), 

    .. code-block:: bash

        git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git

    First follow `the instructions to build the python bindings <https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/#build-python-bindings>`_ from the bitcraze website. Afterwards, make sure that the bindings can be found in the python path:

    .. code-block:: bash

        export PYTHONPATH=<replace-with-path-to>/crazyflie-firmware/build:$PYTHONPATH
        
    If you are working from an older version of the crazyflie-firmware (before tag 2023.02), then you will need to point to main folder of the repo by removing the '/build' part. 


Updating
--------

You can update your local copy using the following commands:

.. code-block:: bash

    cd ros2_ws/src/crazyswarm2
    git pull
    git submodule sync
    git submodule update --init --recursive
    cd ../../
    colcon build --symlink-install


.. Once you have completed installation,
.. move on to the :ref:`configuration` section and configure Crazyswarm for your hardware.
