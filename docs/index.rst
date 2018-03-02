.. Crazyswarm documentation master file, created by
   sphinx-quickstart on Tue Apr 18 20:00:33 2017.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to Crazyswarm's documentation!
======================================

The Crazyswarm projects allows you to fly a swarm of quadcopters (using Bitcraze Crazyflie 2.0 directly or as control boards)
in tight, synchronized formations.
A motion capture system is required (VICON, OptiTrack, PhaseSpace are supported). We successfully flew 49
Crazyflies using three Crazyradios. An example video for what you can do is shown below:

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="https://www.youtube.com/embed/D0CrjoYDt9w" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>



Our contributed code is licensed under the permissive MIT license, however some of the parts (such as the firmware) are licensed under their respective license.

The Crazyswarm architecture, including some motivation for the design decisions, is described in
`our paper <http://usc-actlab.github.io/publications/Preiss_ICRA2017.pdf>`_ [pdf].
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

Contents:

.. toctree::
   changelog
   gettingstarted
   installation
   usage
   api
   hardware
   :maxdepth: 2



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

