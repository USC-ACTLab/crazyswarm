.. _tutorial_streaming_setpoint:

Creating a new streaming setpoint mode
--------------------------------------

In this tutorial, we walk through the steps of implementing a new streaming
setpoint mode. This task is a good tutorial because it touches nearly all parts
of the Crazyswarm system, including the onboard firmware.

.. admonition:: High-level vs. streaming control modes

	In a streaming setpoint mode, the PC sends :term:`Setpoint` values over the radio
	many times per second to each robot individually.
	The onboard feedback controller tries to achieve the states specified by the setpoint.

	In comparison, in a high-level control mode, the PC sends instructions like
	:term:`Piecewise Polynomial` trajectory plans over the radio, and the onboard firmware
	uses the stored plan to compute a new control setpoint in every iteration of
	the main loop.

	Streaming setpoint modes simplify the process of developing new high-level
	planners, since the planner can run on the PC where it has access to more
	computational resources. However, they require more radio bandwidth, so they do
	not scale as well to large numbers of robots. Currently, we have not validated
	anything larger than 7 robots on 3 radios.

	High-level control modes require implementing more planning logic
	onboard in the firmware code. This can be a challenge due to the limited computational
	resources and the difficulty of implementing numerical algorithms in C
	as compared to C++ or Python. However, the effort pays off in lower radio bandwidth
	requirements and increased robustness to hiccups in the radio communication.

The choice of a particular high-level planning algorithm suggests
a choice of which states to include in
the control setpoint. For example, piecewise polynomials (of sufficiently high
order and smoothness) have continuous high-order derivatives, which makes it
possible to compute setpoints for the entire state, including acceleration and
angular velocity. On the other hand, a graph-based planner or RRT might only
give us a sequence of positions connected by straight lines, so it makes sense
to only provide a position setpoint and let the onboard feedback controller
decide whatever velocity, acceleration, attitude, and angular velocity it needs
to track the position setpoint most accurately.

CRTP radio packet definition
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
First, we define our radio packet.
Radio packets follow the CRTP (Crazyflie Real-Time Protocol)
defined by Bitcraze: `<https://www.bitcraze.io/docs/crazyflie-firmware/master/ctrp_index/>`_.
On that page we see the following diagram of the one-byte CRTP header: ::

	   7    6    5    4    3    2    1    0
	+----+----+----+----+----+----+----+----+
	|       Port        |  Link   |  Chan.  |
	+----+----+----+----+----+----+----+----+

The most important part is the 4-bit ``port`` value,
which is used in the Crazyflie firmware to select between different subsystems
for further processing of the packet. Streaming control setpoints use port ``7``.
We define the radio protocol for our setpoint by defining C a struct in 
`crazyflie_cpp/include/crazyflie_cpp/crtp.h <https://github.com/whoenig/crazyflie_cpp/blob/master/include/crazyflie_cpp/crtp.h>`_:

.. code-block:: c

	struct crtpFullStateSetpointRequest
	{
	  crtpFullStateSetpointRequest(
		float x, float y, float z,
		float vx, float vy, float vz,
		float ax, float ay, float az,
		float qx, float qy, float qz, float qw,
		float rollRate, float pitchRate, float yawRate);
	  const crtp header;
	  uint8_t type;
	  int16_t x;
	  int16_t y;
	  int16_t z;
	  int16_t vx;
	  int16_t vy;
	  int16_t vz;
	  int16_t ax;
	  int16_t ay;
	  int16_t az;
	  int32_t quat; // compressed quaternion, xyzw
	  int16_t omegax;
	  int16_t omegay;
	  int16_t omegaz;
	} __attribute__((packed));
	CHECKSIZE(crtpFullStateSetpointRequest)

After the CRTP header, we have another byte ``uint8_t type`` specifying the type of control setpoint.
The canonical source of these values is ``enum packet_type``
`in the Crazyflie firmware <https://github.com/bitcraze/crazyflie-firmware/blob/f28ef7ad675146514caf5388749b466699ba23f3/src/modules/src/crtp_commander_generic.c#L65-L74>`_.
In the remainder of the struct we have 29 bytes left for the setpoint value.

.. admonition:: How we implement the binary protocol

	To send a packet, we cast the struct to a raw byte array ``uint8_t *``.
	The reciever casts the packet bytes back to a struct.
	This technique is not considered robust compared to parsing the byte stream;
	in fact, it only works at all due to the following facts:

	- The C and C++ languages both enforce that a ``struct``'s members are laid out in memory
	  in the same order in which they are declared in the source code.
	- The line ``__attribute((packed))`` is a
	  `GCC extension <https://gcc.gnu.org/onlinedocs/gcc/Common-Type-Attributes.html#Common-Type-Attributes>`_.
	  (This means it is not part of the ANSI C language specification, and C compilers
	  are not required to implement it.) 
	  Its purpose is to disallow the compiler from performing
	  `struct padding <http://www.catb.org/esr/structure-packing/#_padding>`_.
	  This means there will be no empty space in the struct's memory layout.
	  In other words, its ``sizeof`` is exactly the sum of the ``sizeof`` s of its members.
	- The x86 and ARM architectures are both little-endian by default,
	  so types that are larger than 8 bits are decoded in the correct byte order.
	- We always use
	  `exact-width integer types <https://en.wikibooks.org/wiki/C_Programming/stdint.h>`_
	  provided by ``<stdint.h>`` instead of the more familiar types
	  ``int``, ``long``, ``char``, and so on.
	  The latter types
	  `can vary in size on different platforms <https://en.wikipedia.org/wiki/C_data_types#Main_types>`_.
	  For example, ``long`` is 4 bytes on 64-bit Windows systems
	  but 8 bytes on 64-bit Linux systems.

	From all this, one can show that the struct-casting method will produce
	correct results in our setup.
	The macro ``CHECKSIZE`` on the final line uses a ``static_assert``
	to ensure at compile time that the struct is small enough to fit in a radio packet.

Note that our example has used 16-bit fixed-point numbers and advanced
`quaternion <https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation>`_ compression
to fit a lot of values in one packet. Most setpoint types contain fewer values
and fit in the packet without as much compression.
All CRTP structs in ``crtp.h`` should have a constructor that accepts uncompressed types,
so the calling code does not need to know about compression tricks or the exact byte layout.
If there is nontrivial work to be done in the constructor, it can be placed in
``crazyflie_cpp/src/crtp.cpp``, otherwise it should be defined inline in ``crtp.h``.
Our new constructor looks like:

.. code-block:: c++

	crtpFullStateSetpointRequest::crtpFullStateSetpointRequest(
	  float x, float y, float z,
	  float vx, float vy, float vz,
	  float ax, float ay, float az,
	  float qx, float qy, float qz, float qw,
	  float rollRate, float pitchRate, float yawRate)
	  : header(0x07, 0), type(6)
	{
		float s = 1000.0;
		this->x = s * x;
		...
	}

First, we specify port ``7`` in the header byte (as discussed above) and the correct ``type`` value.
In the constructor body, we convert floating-point values in meters
to integer values in millimeters. (Most of the repetitive code is snipped here.)

``crazyflie_cpp`` wrapper
^^^^^^^^^^^^^^^^^^^^^^^^^
Calling code never constructs ``crtp.h`` packets directly;
instead, it interacts with the radio via the ``Crazyflie`` class defined in
`crazyflie_cpp/include/crazyflie_cpp/Crazyflie.h <https://github.com/whoenig/crazyflie_cpp/blob/master/include/crazyflie_cpp/Crazyflie.h>`_.
We need to add a new method to the class ``Crazyflie``:

.. code-block:: c++

  void sendFullStateSetpoint(
    float x, float y, float z,
    float vx, float vy, float vz,
    float ax, float ay, float az,
    float qx, float qy, float qz, float qw,
    float rollRate, float pitchRate, float yawRate);

The implementation should go in
`crazyflie_cpp/src/Crazyflie.cpp <https://github.com/whoenig/crazyflie_cpp/blob/master/src/Crazyflie.cpp>`_
and is very simple:

.. code-block:: c++

	void Crazyflie::sendFullStateSetpoint(
		float x, float y, float z,
		float vx, float vy, float vz,
		float ax, float ay, float az,
		float qx, float qy, float qz, float qw,
		float rollRate, float pitchRate, float yawRate)
	{
	  crtpFullStateSetpointRequest request(
		x, y, z,
		vx, vy, vz,
		ax, ay, az,
		qx, qy, qz, qw,
		rollRate, pitchRate, yawRate);
	  sendPacket(request);
	}


Note that we are using
`the templated overload of Crazyflie::sendPacket <https://github.com/whoenig/crazyflie_cpp/blob/0017a1560b5c14970698ecae98f153701c4518db/include/crazyflie_cpp/Crazyflie.h#L373-L380>`_
that handles casting the struct pointer to ``uint8_t *`` automatically.


ROS service
^^^^^^^^^^^
Although ``crazyflie_cpp`` fully abstracts away the details of the binary
protocol and the radio hardware, there are still a few reasons why it's not
desirable to call ``crazyflie_cpp`` functions directly from higher-level code.

First, the radio transmission functions block due the latency of the USB bus and of
the radio communication itself. This makes sense within ``crazyflie_cpp``,
because there is no other work to do besides sending and recieving radio
packets, and the radio can only be used by one thread at a time.
However, in the higher-level code, there is no reason to block
because control setpoints are a "fire and forget" data stream (like UDP) --
the high-level code should not care whether or not every single control setpoint packet is
transmitted successfully. There is other work to be done instead, such as
computing the next setpoint. This implies that the radio functions should run,
at minimum, in a separate thread.

Additionally, C++ is a good language for a radio protocol implementation,
but it is a more difficult language than Python. A Python wrapper makes
Crazyswarm more accessible to novice programmers.

Finally, if high-level code calls radio functions directly, it cannot run in a
simulator. There needs to be an abstraction layer *somewhere* in the system.

In Crazyswarm, we address these issues by using ROS (Robot Operating System)
to run the ``crazyflie_cpp`` functionality in a separate process.
Despite its name, ROS is not an operating system.
It is an application-level software framework focused on typed interprocess communication.
An introduction to ROS is out of this tutorial's scope;
readers should refer to `<https://wiki.ros.org>`_.
We assume the reader is familiar with ROS's concepts of 
*messages*, *nodes* and *topics*,
which are designed specifically for streaming "fire and forget" data.

Complex setpoint types may require defining a new ROS message type.
It is always preferable to use standard types if an appropriate type exists.
For the full quadrotor state, we define a new message
in `crazyswarm/msg/FullState.msg <https://github.com/USC-ACTLab/crazyswarm/tree/master/ros_ws/src/crazyswarm/msg/FullState.msg>`_:

.. code-block:: none

	Header header
	geometry_msgs/Pose pose
	geometry_msgs/Twist twist
	geometry_msgs/Vector3 acc

Notice how we used the ``Pose`` and ``Twist`` standard compound types instead of raw
``Vector3`` for everything. This helps Crazyswarm's compatibility with other
robotics packages available in ROS, such as planners.

For performance reasons, Crazyswarm is implemented as one monolithic ROS node
instead of several communicating nodes. This means we only need to modify one
class, ``CrazyflieROS``, to support our new setpoint.
There are three main changes: adding a method to handle setpoint messages,
adding a ``ros::Subscriber`` object to subscribe to those messages,
and setting up the subscriber in the ``run()`` method.
We show the basic idea here, abbreviating other methods and repetitive code
with ``...``:

.. code-block:: c++

	class CrazyflieROS
	{
	public:
	...
		void cmdFullStateSetpoint(
			const crazyswarm::FullState::ConstPtr& msg)
		{
			if (!m_isEmergency) {
				float x = msg->pose.position.x;
				...
				float yawRate = msg->twist.angular.z;

				m_cf.sendFullStateSetpoint(x, ..., yawRate);

				m_sentSetpoint = true;
			}
		}
	...
		void run()
		{
			ros::NodeHandle n;
			...
			m_subscribeCmdFullState = n.subscribe(
				m_tf_prefix + "/cmd_full_state",
				1,
				&CrazyflieROS::cmdFullStateSetpoint,
				this);
		}
	
	private:
	...
		ros::Subscriber m_subscribeCmdFullState;
	...
	}

Note that our new method ``cmdFullStateSetpoint`` does little more than unpacking the ROS message
and calling the appropriate method on ``m_cf``, which is an instance of
the ``crazyflie_cpp/Crazyflie`` class. We also check for emergency state.
The flag ``m_sentSetpoint = true`` helps decide whether it is necessary
to send a "heartbeat" ping packet to the Crazyflie.

In the ``run()`` method, we have chosen a name for the setpoint's ROS topic,
and we are initializing our ``ros::Subscriber`` object to connect that topic
to the new method.


``pycrazyswarm`` wrapper
^^^^^^^^^^^^^^^^^^^^^^^^
Finally, we implement the ability to publish the ``FullState`` message in
``pycrazyswarm`` Python class. This is another thin wrapper, taking care of
the ROS publisher object and converting ``numpy`` types into ROS types:

.. code-block:: python

	from crazyswarm.msg import ..., FullState

	class Crazyflie:

		def __init__(...):
			...
			self.cmdFullStatePublisher = rospy.Publisher(
				prefix + "/cmd_full_state", FullState, queue_size=1)
			self.cmdFullStateMsg = FullState()
			self.cmdFullStateMsg.header.seq = 0
			self.cmdFullStateMsg.header.frame_id = "/world"
			...

		...

		def cmdFullState(self, pos, vel, acc, yaw, omega):
			self.cmdFullStateMsg.header.stamp = rospy.Time.now()
			self.cmdFullStateMsg.header.seq += 1
			self.cmdFullStateMsg.pose.position.x    = pos[0]
			...
			self.cmdFullStateMsg.twist.angular.z    = omega[2]
			self.cmdFullStatePublisher.publish(self.cmdFullStateMsg)

The Python class corresponding to the ``FullState`` message has been
auto-generated by ROS's build system ``catkin_make``. We import it here.

In ``class Crazyflie``'s constructor,
we set up a ROS publisher object for our new topic.
Note that we publish on the same topic that we subscribed to in the C++
``CrazyflieROS`` node earlier.
We also construct one ``FullState`` object and store it as a data member.
This is an important performance optimization
to avoid allocating and deallocating heap memory every time we publish a setpoint.

The new method ``cmdFullState(...)`` is the outer layer that ``pycrazyswarm`` scripts will use.
We update the timestamp and sequence number on the message object we constructed earlier;
copy the data from the arguments (either plain Python arrays or ``numpy`` arrays)
into the ROS message object, and finally publish it.
Note that the ``publish(...)`` call will return immediately
rather than waiting for the packet to actually be sent on the radio.

.. admonition:: Note: Why so many layers?

	We have modified three layers on the PC side of things to add our new
	setpoint type: ``crazyflie_cpp``, ``crazyswarm``, and ``pycrazyswarm``.
	We wrote a lot of boilerplate code to copy the same data from
	NumPy types, to ROS types, to C++ function arguments, and finally to
	a CRTP binary protocol struct. To understand what we gained with this
	layered approach, it is helpful to think about the main role of each
	layer:

		1. ``crazyflie_cpp`` is the only layer that needs to understand
		   the radio protocol and how to control the Crazyradio via USB.

		2. ``crazyswarm`` handles all the concurrency.
		   It performs the M:N multiplexing of multiple Crazyflies
		   onto multiple Crazyradios, deals with resending and ACKs
		   in reliable communiation modes (not discussed in this tutorial),
		   communicates with the motion capture system, and so on.

		3. ``pycrazyswarm`` implements the shared abstraction of the
		   real-hardware system and simulator. It is good to do this outside
		   the ROS layer, because ROS is finicky about Linux distributions
		   and versions. We can develop in the simulator on MacOS and other
		   Linuxes.
	
	It is also worth mentioning that ``crazyflie_cpp`` is a standalone 
	project that can be used outside the Crazyswarm setting.


Firmware CRTP parsing
^^^^^^^^^^^^^^^^^^^^^
We are now finished with the PC part of our implementation.
We turn our attention to the onboard firmware.
As mentioned earlier, the first step is to define a packed struct
for "parsing by casting" of the incoming raw bytes.
This takes place entirely in ``crazyflie-firmware/src/modules/src/crtp_commander_generic.c``:

.. code-block:: c

	struct fullStatePacket_s {
		int16_t x;         // position - mm
		int16_t y;
		int16_t z;
		int16_t vx;        // velocity - mm / sec
		int16_t vy;
		int16_t vz;
		int16_t ax;        // acceleration - mm / sec^2
		int16_t ay;
		int16_t az;
		int32_t quat;      // compressed quaternion, see quatcompress.h
		int16_t rateRoll;  // angular velocity - milliradians / sec
		int16_t ratePitch; //  (NOTE: limits to about 5 full circles per sec.
		int16_t rateYaw;   //   may not be enough for extremely aggressive flight.)
	} __attribute__((packed));

We then write a decoder that unpacks the (possibly compressed)
CRTP setpoint packet into the firmware's ``setpoint_t`` struct.
Critically, the ``setpoint_t`` struct contains members for all data
that *any* setpoint mode might require, and "mode" tags that inform the
feedback controller on how it should behave:

.. code-block:: c

	static void fullStateDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
	{
		const struct fullStatePacket_s *values = data;

		ASSERT(datalen == sizeof(struct fullStatePacket_s));

		#define UNPACK(x) \
		setpoint->mode.x = modeAbs; \
		setpoint->position.x = values->x / 1000.0f; \
		setpoint->velocity.x = (values->v ## x) / 1000.0f; \
		setpoint->acceleration.x = (values->a ## x) / 1000.0f; \

		UNPACK(x)
		UNPACK(y)
		UNPACK(z)
		#undef UNPACK

		float const millirad2deg = 180.0f / ((float)M_PI * 1000.0f);
		setpoint->attitudeRate.roll = millirad2deg * values->rateRoll;
		setpoint->attitudeRate.pitch = millirad2deg * values->ratePitch;
		setpoint->attitudeRate.yaw = millirad2deg * values->rateYaw;

		quatdecompress(values->quat, (float *)&setpoint->attitudeQuaternion.q0);
		setpoint->mode.quat = modeAbs;
		setpoint->mode.roll = modeDisable;
		setpoint->mode.pitch = modeDisable;
		setpoint->mode.yaw = modeDisable;
	}

The ``UNPACK`` macro is a questionable attempt to reduce the amount of boilerplate code.
It may be removed in the future.
(This function would be much simpler if the packet were not compressed.)
Within ``UNPACK``, setting ``setpoint->mode->x`` to ``modeAbs`` informs the
controller that it should track the absolute position, not just velocity.
The other modes
(defined in ``crazyflie-firmware/src/modules/interface/stabilizer-types.h``)
are ``modeVelocity``, for velocity tracking,
and ``modeDisable``, meaning the controller should ignore that state completely.

Next, we add our new setpoint to the ``packet_type`` enum
and map this particular enum value to our decoder via an array of function pointers:

.. code-block:: c

	enum packet_type {
		...
		fullStateType = 6,
		...
	};

	...

	const static packetDecoder_t packetDecoders[] = {
		...
		[fullStateType]		 = fullStateDecoder,
		...
	};

Note that the value ``6`` for this enum corresponds to the initialization
of the ``uint8_t type`` member of ``crtpFullStateSetpointRequest`` in
its constructor in
``crazyflie_cpp/.../crtp.h``. It is the programmer's job to ensure these match.

The function ``crtpCommanderGenericDecodeSetpoint`` parses the ``type`` byte
and dispatches to the correct decoder.
By construction, it does not need to be modified when we add a new setpoint type.


Onboard control
^^^^^^^^^^^^^^^
The needed changes in onboard feedback control may change depending on the
semantics of the new setpoint. In the case of the full-state setpoint,
it is intended to be used with the "Mellinger" controller
(`crazyflie-firmware/src/modules/src/controller_mellinger.c <https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/controller_mellinger.c>`_,
named after the paper by Daniel Mellinger and Vijay Kumar).

A notable snippet of the controller code is the following:

.. code-block:: c

	if (setpoint->mode.x == modeAbs) {
		target_thrust.x = g_vehicleMass * setpoint->acceleration.x                       + kp_xy * r_error.x + kd_xy * v_error.x + ki_xy * i_error_x;
		target_thrust.y = g_vehicleMass * setpoint->acceleration.y                       + kp_xy * r_error.y + kd_xy * v_error.y + ki_xy * i_error_y;
		target_thrust.z = g_vehicleMass * (setpoint->acceleration.z + GRAVITY_MAGNITUDE) + kp_z  * r_error.z + kd_z  * v_error.z + ki_z  * i_error_z;
	} else {
		target_thrust.x = -sinf(radians(setpoint->attitude.pitch));
		target_thrust.y = -sinf(radians(setpoint->attitude.roll));
		// In case of a timeout, the commander tries to level, ie. x/y are disabled, but z will use the previous setting
		// In that case we ignore the last feedforward term for acceleration
		if (setpoint->mode.z == modeAbs) {
		  target_thrust.z = g_vehicleMass * GRAVITY_MAGNITUDE + kp_z  * r_error.z + kd_z  * v_error.z + ki_z  * i_error_z;
		} else {
		  target_thrust.z = 1;
		}
	}

The ``if`` statement checks the ``mode`` values in the setpoint to see if the
setpoint has specified absolute position control or not.
If so (the first branch), we compute a target thrust vector using a PID  control law
with the desired position and velocity from the setpoint. We also use the
acceleration value from the setpoint as a feedforward term.
If not, we construct a target thrust vector based on the roll and pitch setpoints.

This code does not seem to handle the case when ``setpoint->mode.x == modeVelocity``.
Presumably, the second branch is only meant to handle ``setpoint->mode.x == modeDisable``.
In general, the "compatibility matrix" between setpoint types and controller types
(other controllers include ``controller_pid.c`` and ``controller_indi.c``)
is somewhat murky; this should be improved in future work.

