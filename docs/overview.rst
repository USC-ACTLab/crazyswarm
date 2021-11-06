.. _overview:

Overview
========

Crazyswarm has the following software architecture.

    .. figure:: images/software_architecture.png
        :align: center
        :scale: 70%


- **crazyflie_tools**
  These are command line tools that can be used using `rosrun crazyflie_tools <name> <arguments>`. These tools include methods to list logging variables, parameters, and to reboot individual crazyflies.
- **crazyswarm_server**
  This application is the core of the Crazyswarm. It provides the ROS interface, communicates with the robots and the motion capture system.
- **pycrazyswarm**
  This is a simplified Python library to use the Crazyswarm. It has two backends: the physical backend (communicating with the crazyswarm_server) and the simulation backend. The simulator uses parts of the official firmware for software-in-the-loop simulation. For performance reasons, the simulation does not include the dynamics and rather visualizes the setpoints.
- **Helper libraries**
  We provide a unified interface for different motion capture systems (`libMotionCapture`), a way to track rigid bodies frame-by-frame even with unique marker configurations (`libObjectTracker`), and a library for the low-level communication with the Crazyflie robots (`crazyflie_cpp`).

More Information
----------------

- Talk at the `BAM days 2021 <https://www.bitcraze.io/about/events/bam2021/>`__
    * `Slides <https://www.bitcraze.io/about/events/documents/bam2021/hoenig_crazyswarm_bam2021.pdf>`__ [pdf].
    * `Video <https://youtu.be/9KlfFpv6NIQ>`__ [youtube].
- Talk at the `Aerial Swarms Workshop <https://lis2.epfl.ch/iros2019swarms/index.html>`__ at IROS 2019
    * `Slides <https://drive.google.com/file/d/15favAyrLLpC_O6nrAp-eIbZijFUMLgwV/view?usp=sharing>`__ [pdf].
