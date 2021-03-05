Crazyswarm Integration with Git
-------------------------------

In this tutorial we discuss ways to use the Crazyswarm in your own projects, while properly 
version-controlling you custom launch files and scripts.


Option 1: Fork
^^^^^^^^^^^^^^

The most straight-forward approach is to fork the Crazyswarm repository. This fork can be
included as a submodule in your own projects and your own launch files and scripts can
be in `ros_ws/src/crazyswarm` along with the provided examples.

The only downside of this approach is that it becomes rather difficult to update with the 
latest version upstream, especially if you have changes in one of the submodules (like 
the firmware).

Option 2: Custom ROS Package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you only have custom launch files, scripts, or firmware, you can simplify the process
by creating a custom out-of-source ROS package: ::


	any/folder$ catkin_create_pkg my-crazyswarm-pkg

In particular, you can create this package in any folder, including another git repository.

Then, add this package to your crazyswarm workspace by using a symbolic link: ::

	crazyswarm/ros_ws/src/userPackages$ ln -s /path/to/my-crazyswarm-pkg .

In you package, you can have your own launch files, yaml configuration files, and scripts,
similar to the folder structure in the crazyswarm package.

To run `chooser.py`, you can use its optional arguments::

	crazyswarm/ros_ws/crazyswarm/scripts$ python chooser.py --configpath ../../userPackages/my-crazyswarm-pkg/launch/ --stm32Fw /path/to/cf2.bin

To run your own launch file, simply use: ::

	$ roslaunch my-crazyswarm-pkg my-custom-launch-file.launch

To run your own scripts, you need to adjust the Python package search path so that our
helper library is found: ::

	my-crazyswarm-pkg/scripts$ export PYTHONPATH=$PYTHONPATH:/path/to/crazyswarm/ros_ws/src/crazyswarm/scripts
	my-crazyswarm-pkg/scripts$ examplescript.py
