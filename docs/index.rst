.. _introduction:

Welcome to Crazyswarm's documentation!
======================================

The Crazyswarm platform allows you to fly a swarm of
`Bitcraze Crazyflie 2.x <https://www.bitcraze.io/products/crazyflie-2-1/>`_
quadcopters in tight, synchronized formations.
A motion capture system is recommended: VICON, OptiTrack, and Qualisys are supported.
We successfully flew 49 Crazyflies using three Crazyradios.
An example video for what you can do is shown below:

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; margin-bottom: 20pt; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="https://www.youtube.com/embed/D0CrjoYDt9w" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>

It is also possible to use the Crazyflie as a control board for other quadcopter hardware.


How is Crazyswarm different from Bitcraze's `Crazyflie Python API <https://github.com/bitcraze/crazyflie-lib-python>`_?
-----------------------------------------------------------------------------------------------------------------------

Both can be used to control several Crazyflies from a Python script.
Here are some differences:

- **Motion capture integration.**
  Crazyswarm contains drivers for common motion capture systems.
  The Bitcraze API can *send* position measurements to the Crazyflie,
  but does not know how to *get* position measurements from mocap hardware.
- **Identical or single motion capture markers.**
  Via `libobjecttracker <https://github.com/USC-ACTLab/libobjecttracker>`_,
  Crazyswarm can track multiple quadrotors with identical motion capture marker arrangements,
  or quadrotors with only one marker each.
  Most motion capture devices do not support this natively.
  To make it possible, the user must supply the quadrotors' initial positions in a configuration file
  at startup to establish the mapping from radio addresses to positions.
- **Simulation.**
  Crazyswarm has a simulation mode with 3D graphics,
  which makes it easy to validate complex scripts before running them on real hardware.
- **Python firmware bindings.**
  Crazyswarm's simulator is built upon automatically generated Python bindings
  for certain modules in the Crazyflie firmware.
  The binding system can be helpful when developing new firmware modules,
  especially when they are mathematically complex and hard to debug.
- **ROS foundation.**
  The Crazyswarm server program is a ROS node.
  The :ref:`api` is a thin wrapper around the ROS interface.
  While we recommend the Python API for most applications,
  the ROS interface is fully supported.


Crazyswarm's academic origins
-----------------------------

The Crazyswarm architecture, including some motivation for the design decisions, is described in
`our paper <http://usc-actlab.github.io/publications/Preiss_ICRA2017.pdf>`_ [pdf].

A talk at the `Aerial Swarms Workshop <https://lis2.epfl.ch/iros2019swarms/index.html>`_ at IROS 2019 includes a primer on how to use the Crazyswarm and a bibliography of papers using the Crazyswarm: `Slides <https://drive.google.com/file/d/15favAyrLLpC_O6nrAp-eIbZijFUMLgwV/view?usp=sharing>`_ [pdf].


If you use our work in academic research, please cite us:

.. code-block:: none

    @inproceedings{crazyswarm,
      author    = {James A. Preiss* and
                   Wolfgang  H\"onig* and
                   Gaurav S. Sukhatme and
                   Nora Ayanian},
      title     = {Crazyswarm: {A} large nano-quadcopter swarm},
      booktitle = {{IEEE} International Conference on Robotics and Automation ({ICRA})},
      pages     = {3299--3304},
      publisher = {{IEEE}},
      year      = {2017},
      url       = {https://doi.org/10.1109/ICRA.2017.7989376},
      doi       = {10.1109/ICRA.2017.7989376},
      note      = {Software available at \url{https://github.com/USC-ACTLab/crazyswarm}},
    }


Our contributed code is licensed under the permissive MIT license, however some of the parts (such as the firmware) are licensed under their respective license.


Contents
--------

.. toctree::
   changelog
   gettingstarted
   installation
   configuration
   tutorials/tutorials
   howto/howto
   api
   internals
   hardware
   glossary
   :maxdepth: 2



Indices and tables
------------------

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

