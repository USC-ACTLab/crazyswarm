.. _installation:

Installation
============

Crazyswarm2 runs on **Ubuntu Linux** in one of the following configurations:

====== ====== ========
Ubuntu Python ROS2
------ ------ --------
20.04  3.7    Galactic
====== ====== ========

.. warning::
   The `Windows Subsystem for Linux (WSL) <https://docs.microsoft.com/en-us/windows/wsl/about>`_ is not supported.
   You must install Ubuntu either directly on the computer (recommended) or in a VM.

.. warning::
   Avoid using a virtual machine if possible: they add additional latency and might cause issues with the visualization tools.

First Installation
------------------

1. If needed, install ROS2 using the instructions at https://docs.ros.org/en/galactic/Installation.html.

2. Set up your ROS2 workspace

    .. code-block:: bash

        mkdir -p ros2_ws/src
        cd ros2_ws/src
        git clone https://github.com/IMRCLab/crazyswarm2 --recursive
        git clone --branch ros2 --recursive https://github.com/IMRCLab/motion_capture_tracking.git

3. Build your ROS2 workspace

    .. code-block:: bash

        cd ../
        colcon build --symlink-install

    .. note::
       symlink-install allows you to edit Python and config files without running `colcon build` every time.

4. Set up software-in-the-loop simulation

    .. code-block:: bash

        cd py_crazyswarm2/py_crazyswarm2/cfsim
        make


Updating
--------

You can update your local copy using the following commands:

.. code-block:: bash

    cd ros2_ws/src/crazyswarm2
    git pull
    git submodule sync
    git submodule update --init --recursive
    cd ../
    colcon build --symlink-install


.. Once you have completed installation,
.. move on to the :ref:`configuration` section and configure Crazyswarm for your hardware.
