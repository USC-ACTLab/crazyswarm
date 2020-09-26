Crazyswarm Internals
====================

This page contains information for developers interested in modifying the Crazyswarm platform.
Developers interested in using Crazyswarm as a library/framework should refer to :ref:`api` instead.

The tutorial :ref:`tutorial_streaming_setpoint` contains a significant amount of design motivation and internal details.
It is worth reading even if you are not working on streaming setpoints.

Firmware bindings
-----------------
Selected modules from the Crazyflie firmware are exposed to ``pycrazyswarm`` scripts as a Python module via bindings.
The bindings are automatically generated from the C header files using 
`SWIG <http://www.swig.org/>`_.
They are used extensively in
`crazyflieSim.py <https://github.com/USC-ACTLab/crazyswarm/blob/master/ros_ws/src/crazyswarm/scripts/pycrazyswarm/crazyflieSim.py>`_
to mimic the behavior of a real Crazyflie in simulation mode.

Debugging firmware modules via bindings
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. highlight:: none

The bindings are useful for testing and implementing new Crazyflie firmware modules.
If the core functionality of the module is written without any dependencies on ARM or FreeRTOS,
it can be compiled for x86 and tested/debugged on a PC.

Linux / GDB
^^^^^^^^^^^
On Linux, bindings are compiled with GCC, so the preferred debugger is GDB.
To invoke a Python script with arguments, use the ``--args`` flag.
If your script is a unit test, GDB does not recognize the ``pytest`` command, so use::

    gdb --args python -m pytest test_highLevel.py

After GDB starts, you can place a breakpoint inside a firmware C file::

    (gdb) b planner.c:plan_takeoff

Although the symbol is not loaded yet, GDB is able to resolve the breakpoint later
when the Python module is imported. Enter ``y`` at  the prompt::

    No source file named planner.c.
    Make breakpoint pending on future shared library load? (y or [n]) y
    Breakpoint 1 (planner.c:plan_takeoff) pending.

Then, when we type the ``run`` command, the Python interpreter runs for a while and hits our breakpoint::

    (gdb) run
    ...
    Breakpoint 1, plan_takeoff (p=p@entry=0x555557522cd0, ...)
    at ../../../../../../crazyflie-firmware/src/modules/src/planner.c:135

The first time you run GDB, it's useful to enter the command::

    (gdb) set history save on

This will save your commands into a file ``.gdb_history`` in the working directory.
Next time you run GDB, the commands from the previous run will still be available
using the "up" arrow key.

MacOS / LLDB
^^^^^^^^^^^^
On MacOS, bindings are compiled with Clang, so LLDB is the preferred debugger.
LLDB is not a drop-in replacement for GDB; the commands are different.

To invoke a Python script with arguments, use ``--``::

    lldb -- python -m pytest test_highLevel.py

After LLDB starts, the equivalent syntax to place a breakpoint in a firmware C file is::

    (lldb) br set -f planner.c -n plan_takeoff

LLDB will automatically set up the pending breakpoint and resolve it upon library loading.
Unlike GDB, it will not ask for your confirmation.
When we type the ``run`` command, the Python interpreter runs for a while and hits our breakpoint::

    (lldb) run
    ...
    * thread #1, queue = 'com.apple.main-thread', stop reason = breakpoint 1.1
        frame #0: 0x000000010f554d10 _cffirmware.so`plan_takeoff(p=0x0000000100780bd0, ...)
        at planner.c:136:15 [opt]

LLDB automatically preserves command history between runs without any command.
