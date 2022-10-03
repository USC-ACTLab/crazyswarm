.. _tutorials:

Tutorials
=========

Mapping with the SLAM toolbox
-----------------------------

You can connect the Crazyflie through ROS2 with existing packages like the `SLAM toolbox <https://github.com/SteveMacenski/slam_toolbox/>`_. 
With a `Flow deck <https://www.bitcraze.io/products/flow-deck-v2/>`_ and `Multi-ranger <https://www.bitcraze.io/products/multi-ranger-deck/>`_
) a simple map can be created.

Mind that this will only show the mapping, as the ray matching with the sparse multiranger is quite challenging for the SLAM toolbox. 

Preperation
~~~~~~~~~~~

Assuming you have installed ROS2 and Crazyswarm2 according to the instructions and went through the guides on Usage, now install the slam toolbox:

.. code-block:: bash

    sudo apt-get install ros-galactic-slam-toolbox

Go to crazyflie/config/crazyflie.yaml, change the URI of the crazyflie to the one yours has:

.. code-block:: bash

  cf1:
    enabled: true
    uri: radio://0/20/2M/E7E7E7E701

And enable the following default topic logging:

.. code-block:: bash

  firmware_logging:
    enabled: true
    default_topics:
      odom:
        frequency: 10 # Hz
      scan:
        frequency: 10 # Hz

Also make sure that that the standard controller is set to 1 (PID) for the flowdeck and the state estimator is set to 2 (kalman):

.. code-block:: bash

  firmware_params:
    stabilizer:
      estimator: 2 # 1: complementary, 2: kalman
      controller: 1 # 1: PID, 2: mellinger

Connecting with the Crazyflie
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Turn on your crazyflie and put it in the middle of the room you would like to map.

Now startup the crazyflie server with the following example launch file, after sourcing the setup.bash ofcourse:

.. code-block:: bash

    source install/setup.bash
    ros2 launch crazyflie_examples multiranger_mapping_launch.py 

You should now see the M4 LED blinking green and red and the following appear on the screen:

.. code-block:: bash

    [INFO] [launch]: All log files can be found below /home/knmcguire/.ros/log/2022-10-03-16-15-53-553693-kim-legion-15498
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [crazyflie_server.py-1]: process started with pid [15500]
    [INFO] [vel_mux.py-2]: process started with pid [15502]
    [INFO] [async_slam_toolbox_node-3]: process started with pid [15504]
    [async_slam_toolbox_node-3] [INFO] [1664806553.866149124] [slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
    [vel_mux.py-2] [INFO] [1664806559.174521891] [vel_mux]: Velocity Multiplexer set for /cf1 with height 0.3 m using the /cmd_vel topic
    [crazyflie_server.py-1] [INFO] [1664806560.043101845] [crazyflie_server]:  radio://0/20/2M/E7E7E7E701 is fully connected!
    [crazyflie_server.py-1] [INFO] [1664806560.044138096] [crazyflie_server]: All Crazyflies are fully connected!
    [crazyflie_server.py-1] [INFO] [1664806560.054259470] [crazyflie_server]:  radio://0/20/2M/E7E7E7E701: commander.enHighLevel is set to 1
    [crazyflie_server.py-1] [INFO] [1664806560.105691178] [crazyflie_server]:  radio://0/20/2M/E7E7E7E701: stabilizer.controller is set to 1
    [crazyflie_server.py-1] [INFO] [1664806560.107138259] [crazyflie_server]:  radio://0/20/2M/E7E7E7E701: stabilizer.estimator is set to 2
    [crazyflie_server.py-1] [INFO] [1664806560.114968490] [crazyflie_server]: All Crazyflies parameters are initialized
    [crazyflie_server.py-1] [INFO] [1664806560.116479518] [crazyflie_server]: radio://0/20/2M/E7E7E7E701 setup logging for scan at freq 10
    [crazyflie_server.py-1] [INFO] [1664806560.118522365] [crazyflie_server]: radio://0/20/2M/E7E7E7E701 setup logging for odom at freq 10
    [crazyflie_server.py-1] [INFO] [1664806560.123137907] [crazyflie_server]: All Crazyflies loggging are initialized
    [async_slam_toolbox_node-3] [INFO] [1664806560.329904109] [slam_toolbox]: Message Filter dropping message: frame 'cf1' at time 1664806560.232 for reason 'discarding message because the queue is full'
    [async_slam_toolbox_node-3] Info: clipped range threshold to be within minimum and maximum range!
    [async_slam_toolbox_node-3] [WARN] [1664806560.333439709] [slam_toolbox]: maximum laser range setting (3.5 m) exceeds the capabilities of the used Lidar (3.5 m)
    [async_slam_toolbox_node-3] Registering sensor: [Custom Described Lidar]


If anything is off, check if the crazyflie.yaml has been configured correctly!

Now, open up a  rviv2 window in a seperate terminal with :

.. code-block:: bash

    source /opt/ros/galactic/setup.bash
    rviz2

Add the following displays and panels to RVIZ:
* Changed the 'Fixed frame' to 'world
* 'Add' button under displays and 'by topic' tab, select the '/map' topic.
* 'Add' button under displays and 'by display type' add a transform.
* 'Panels' on the top menu, select 'add new panel' and select the SLAMToolBoxPlugin

It should look like something like this:

.. image:: images/slam_rviz2.jpg


Flying and mapping
~~~~~~~~~~~~~~~~~~

While still connected to the crazyflie with the server, open another terminal and type:

.. code-block:: bash

    source /opt/ros/galactic/setup.bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

and make the crazyflie take off with the 't' key on your keyboard. Now fly around the room to make a map of it.

Tip: start with turning slowely with yaw, which should be enough to get most of the room. 


Once you are happy, you can save the map with 'Save Map' in the SLAM toolbox panel, and land the crazyflie with 't'










