.. _installation:

Installation
============

Crazyswarm runs on **Ubuntu Linux** in one of the following configurations:

====== ====== =======
Ubuntu Python ROS
------ ------ -------
20.04  3.7    Noetic
18.04  2.7    Melodic
====== ====== =======

For simulation-only operation, **Mac OS** is also supported.
Click the appropriate tab(s) below to see the installation instructions for your desired configuration.

.. note::
   You must set the environment variable ``$CSW_PYTHON`` to the name of your Python interpreter
   (e.g. ``python2`` or ``python3``)
   before building.


.. warning::
   The `Windows Subsystem for Linux (WSL) <https://docs.microsoft.com/en-us/windows/wsl/about>`_ is not supported.
   You must install Ubuntu either directly on the computer (recommended) or in a VM.


.. tabs::

   .. tab:: Simulation Only

      You can write/debug ``pycrazyswarm`` scripts on a machine that does not have ROS installed.
      On Mac OS, Crazyswarm must run within an Anaconda environment.
      On Linux, using Anaconda is optional.
      Select your platform from the tabs below:

      .. tabs::

         .. tab:: Linux or Mac OS with Anaconda

            1. Set the ``$CSW_PYTHON`` environment variable::

                $ export CSW_PYTHON=[python2 or python3]

            2. Install `Anaconda Python 2.7 / 3.7 version <https://www.anaconda.com/distribution>`_ (We have tested on version ``2019.10``).

            3. Clone the Crazyswarm git repository::

                $ git clone https://github.com/USC-ACTLab/crazyswarm.git

            4. Create the Anaconda environment with your desired Python version::

                $ cd crazyswarm
                $ conda create --name crazyswarm python=[2.7 or 3.7]
                $ conda env update -f conda_env.yaml

            5. Activate the Anaconda environment::

                $ conda activate crazyswarm

            6. Run the build script::

                $ ./buildSimOnly.sh

            7. Verify the installation by running the unit tests::

                $ cd ros_ws/src/crazyswarm/scripts
                $ source ../../../devel/setup.bash
                $ $CSW_PYTHON -m pytest

         .. tab:: Linux without Anaconda

            1. Set the ``$CSW_PYTHON`` environment variable::

                $ export CSW_PYTHON=[python2 or python3]

            2. Install the dependencies

               .. program-output:: python3 generate_install_deps_code.py ../.github/workflows/ci-ros.yml | sed -e '/ros/d' -e '/usb/d'
                  :shell:

            3. Install one or more visualizers for simulation::

                (Recommended)
                $ ${CSW_PYTHON} -m pip install vispy
                (Alternative)
                $ ${CSW_PYTHON} -m pip install matplotlib

            4. [OPTIONAL] Install ffmpeg if you want to record high-quality videos from the simulator::

                $ sudo apt install -y ffmpeg
                $ ${CSW_PYTHON} -m pip install ffmpeg-python

            5. Clone the Crazyswarm git repository::

                $ git clone https://github.com/USC-ACTLab/crazyswarm.git

            6. Run the build script::

                $ cd crazyswarm
                $ ./buildSimOnly.sh

            7. Verify the installation by running the unit tests::

                $ cd ros_ws/src/crazyswarm/scripts
                $ source ../../../devel/setup.bash
                $ $CSW_PYTHON -m pytest


   .. tab:: Physical Robots and Simulation

      For real hardware operation, ensure that your platform matches
      one of the configurations in the table above.
      **Avoid using a virtual machine** if possible:
      they add additional latency and might cause issues with the visualization tools.

      1. If needed, install ROS using the instructions at http://wiki.ros.org/ROS/Installation.

      2. Set the ``$CSW_PYTHON`` environment variable::

          $ export CSW_PYTHON=[python2 or python3]

      3. Install the dependencies

         .. program-output:: python3 generate_install_deps_code.py ../.github/workflows/ci-ros.yml
            :shell:

      4. Install one or more visualizers for simulation::

          (Recommended)
          $ ${CSW_PYTHON} -m pip install vispy
          (Alternative)
          $ ${CSW_PYTHON} -m pip install matplotlib

      5. [OPTIONAL] Install ffmpeg if you want to record high-quality videos from the simulator::

          $ sudo apt install -y ffmpeg
          $ ${CSW_PYTHON} -m pip install ffmpeg-python

      6. Clone the Crazyswarm git repository::

          $ git clone https://github.com/USC-ACTLab/crazyswarm.git

      7. Run the build script::

          $ cd crazyswarm
          $ ./build.sh

      8. Verify the installation by running the unit tests::

          $ cd ros_ws/src/crazyswarm/scripts
          $ source ../../../devel/setup.bash
          $ $CSW_PYTHON -m pytest


Once you have completed installation,
move on to the :ref:`configuration` section and configure Crazyswarm for your hardware.
