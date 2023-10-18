.. _introduction:

Crazyswarm2: A ROS 2 testbed for Aerial Robot Teams
===================================================

Crazyswarm2 is a **work-in-progress** port of the original `Crazyswarm <https://crazyswarm.readthedocs.io>`_ to ROS 2.
It is fully open-source and available on `github <https://github.com/IMRCLab/crazyswarm2>`_.

Crazyswarm2 is primarily made for researchers that want to operate or simulate a team of unmanned aerial vehicles (UAVs) that uses the
`Bitcraze Crazyflie 2.x <https://www.bitcraze.io/products/crazyflie-2-1/>`_ or `Bitcraze Crazyflie Bolt-based <https://store.bitcraze.io/products/crazyflie-bolt>`_ UAVs.

.. warning::
  Crazyswarm2 is already usable for basic tasks (teleoperation, flying in a motion capture space, data collection), however the API and configuration file formats might still change. While many features of Crazyswarm1 are ported, there are currently still some important limitations:
  
  - Tested only for small team sizes (less than 10 robots)
  - Limited Python API and only a few example scripts

.. Crazyswarm's academic origins
.. -----------------------------

.. The Crazyswarm architecture, including some motivation for the design decisions, is described in
.. `our paper <http://usc-actlab.github.io/publications/Preiss_ICRA2017.pdf>`_ [pdf].

.. A talk at the `Aerial Swarms Workshop <https://lis2.epfl.ch/iros2019swarms/index.html>`_ at IROS 2019 includes a primer on how to use the Crazyswarm and a bibliography of papers using the Crazyswarm: `Slides <https://drive.google.com/file/d/15favAyrLLpC_O6nrAp-eIbZijFUMLgwV/view?usp=sharing>`_ [pdf].


.. If you use our work in academic research, please cite us:

.. .. code-block:: none

..     @inproceedings{crazyswarm,
..       author    = {James A. Preiss* and
..                    Wolfgang  H\"onig* and
..                    Gaurav S. Sukhatme and
..                    Nora Ayanian},
..       title     = {Crazyswarm: {A} large nano-quadcopter swarm},
..       booktitle = {{IEEE} International Conference on Robotics and Automation ({ICRA})},
..       pages     = {3299--3304},
..       publisher = {{IEEE}},
..       year      = {2017},
..       url       = {https://doi.org/10.1109/ICRA.2017.7989376},
..       doi       = {10.1109/ICRA.2017.7989376},
..       note      = {Software available at \url{https://github.com/USC-ACTLab/crazyswarm}},
..     }


.. Our contributed code is licensed under the permissive MIT license, however some of the parts (such as the firmware) are licensed under their respective license.


Contents
--------

.. toctree::
   installation
   overview
   usage
   tutorials
   howto
   faq
   :maxdepth: 1


.. Indices and tables
.. ------------------

.. * :ref:`genindex`
.. * :ref:`modindex`
.. * :ref:`search`

