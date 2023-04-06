#!/usr/bin/env python


# import sys
# import rospy
import numpy as np
# import time
# import tf_conversions
# from std_srvs.srv import Empty
# import std_msgs
# from crazyswarm.srv import *
# from crazyswarm.msg import TrajectoryPolynomialPiece, FullState, Position, VelocityWorld
# from tf import TransformListener
# from .visualizer import visNull

from collections import defaultdict

import rclpy
import rclpy.node
import rowan
from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Twist
from rcl_interfaces.srv import GetParameters, SetParameters, ListParameters, DescribeParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from crazyflie_interfaces.srv import Takeoff, Land, GoTo, UploadTrajectory, StartTrajectory, NotifySetpointsStop
from crazyflie_interfaces.msg import TrajectoryPolynomialPiece, FullState, Position

def arrayToGeometryPoint(a):
    result = Point()
    result.x = a[0]
    result.y = a[1]
    result.z = a[2]
    return result

class TimeHelper:
    """Object containing all time-related functionality.

    This class mainly exists to support both real hardware and (potentially
    faster or slower than realtime) simulation with the same script.
    When running on real hardware, this class uses ROS time functions.
    The simulation equivalent does not depend on ROS.

    Attributes:
        visualizer: No-op object conforming to the Visualizer API used in
            simulation scripts. Maintains the property that scripts should not
            know/care if they are running in simulation or not.
    """
    def __init__(self, node):
        self.node = node
        # self.rosRate = None
        self.rateHz = None
        self.nextTime = None
        # self.visualizer = visNull.VisNull()

    def time(self):
        """Returns the current time in seconds."""
        return self.node.get_clock().now().nanoseconds / 1e9

    def sleep(self, duration):
        """Sleeps for the provided duration in seconds."""
        start = self.time()
        end = start + duration
        while self.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0)

    def sleepForRate(self, rateHz):
        """Sleeps so that, if called in a loop, executes at specified rate."""
        # Note: The following ROS 2 construct cannot easily be used, because in ROS 2
        #       there is no implicit threading anymore. Thus, the rosRate.sleep() call
        #       is blocking. Instead, we simulate the rate behavior ourselves.
        # if self.rosRate is None or self.rateHz != rateHz:
        #     self.rosRate = self.node.create_rate(rateHz)
        #     self.rateHz = rateHz
        # self.rosRate.sleep()
        if self.nextTime is None or self.rateHz != rateHz:
            self.rateHz = rateHz
            self.nextTime = self.time() + 1.0 / rateHz
        while self.time() < self.nextTime:
            rclpy.spin_once(self.node, timeout_sec=0)
        self.nextTime += 1.0 / rateHz

    def isShutdown(self):
        """Returns true if the script should abort, e.g. from Ctrl-C."""
        return not rclpy.ok()


class Crazyflie:
    """Object representing a single robot.

    The bulk of the module's functionality is contained in this class.
    """

    def __init__(self, node, cfname, paramTypeDict):
        """Constructor.

        Args:
            cfname (string): Name of the robot names[ace]
        """
        prefix = "/" + cfname
        self.prefix = prefix
        self.node = node

        # self.tf = tf

        # rospy.wait_for_service(prefix + "/set_group_mask")
        # self.setGroupMaskService = rospy.ServiceProxy(prefix + "/set_group_mask", SetGroupMask)
        self.emergencyService = node.create_client(Empty, prefix + "/emergency")
        self.emergencyService.wait_for_service()
        self.takeoffService = node.create_client(Takeoff, prefix + "/takeoff")
        self.takeoffService.wait_for_service()
        self.landService = node.create_client(Land, prefix + "/land")
        self.landService.wait_for_service()
        # # rospy.wait_for_service(prefix + "/stop")
        # # self.stopService = rospy.ServiceProxy(prefix + "/stop", Stop)
        self.goToService = node.create_client(GoTo, prefix + "/go_to")
        self.goToService.wait_for_service()
        self.uploadTrajectoryService = node.create_client(UploadTrajectory, prefix + "/upload_trajectory")
        self.uploadTrajectoryService.wait_for_service()
        self.startTrajectoryService = node.create_client(StartTrajectory, prefix + "/start_trajectory")
        self.startTrajectoryService.wait_for_service()
        self.notifySetpointsStopService = node.create_client(NotifySetpointsStop, prefix + "/notify_setpoints_stop")
        self.notifySetpointsStopService.wait_for_service()
        self.setParamsService = node.create_client(SetParameters, "/crazyflie_server/set_parameters")
        self.setParamsService.wait_for_service()

        # Query some settings
        getParamsService = node.create_client(GetParameters, "/crazyflie_server/get_parameters")
        getParamsService.wait_for_service()
        req = GetParameters.Request()
        req.names = ["robots.{}.initial_position".format(cfname), "robots.{}.uri".format(cfname)]
        future = getParamsService.call_async(req)
        while rclpy.ok():
            rclpy.spin_once(node)
            if future.done():
                response = future.result()
                # extract initial position
                if response.values[0].type == ParameterType.PARAMETER_INTEGER_ARRAY:
                    self.initialPosition = np.array(response.values[0].integer_array_value)
                elif response.values[0].type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                    self.initialPosition = np.array(response.values[0].double_array_value)
                else:
                    assert(False)

                # extract uri
                self.uri = response.values[1].string_value

                break

        self.paramTypeDict = paramTypeDict

        self.cmdFullStatePublisher = node.create_publisher(FullState, prefix + "/cmd_full_state", 1)
        self.cmdFullStateMsg = FullState()
        self.cmdFullStateMsg.header.frame_id = "/world"

        # self.cmdStopPublisher = rospy.Publisher(prefix + "/cmd_stop", std_msgs.msg.Empty, queue_size=1)

        # self.cmdVelPublisher = rospy.Publisher(prefix + "/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)

        self.cmdPositionPublisher = node.create_publisher(Position, prefix + "/cmd_position", 1)
        self.cmdPositionMsg = Position()
        self.cmdPositionMsg.header.frame_id = "/world"


        # self.cmdVelocityWorldPublisher = rospy.Publisher(prefix + "/cmd_velocity_world", VelocityWorld, queue_size=1)
        # self.cmdVelocityWorldMsg = VelocityWorld()
        # self.cmdVelocityWorldMsg.header.seq = 0
        # self.cmdVelocityWorldMsg.header.frame_id = "/world"

    # def setGroupMask(self, groupMask):
    #     """Sets the group mask bits for this robot.

    #     The purpose of groups is to make it possible to trigger an action
    #     (for example, executing a previously-uploaded trajectory) on a subset
    #     of all robots without needing to send more than one radio packet.
    #     This is important to achieve tight, synchronized "choreography".

    #     Up to 8 groups may exist, corresponding to bits in the groupMask byte.
    #     When a broadcast command is triggered on the :obj:`CrazyflieServer` object
    #     with a groupMask argument, the command only affects those robots whose
    #     groupMask has a nonzero bitwise-AND with the command's groupMask.
    #     A command with a groupMask of zero applies to all robots regardless of
    #     group membership.

    #     Some individual robot (not broadcast) commands also support groupMask,
    #     but it is not especially useful in that case.

    #     Args:
    #         groupMask (int): An 8-bit integer representing this robot's
    #             membership status in each of the <= 8 possible groups.
    #     """
    #     self.setGroupMaskService(groupMask)

    # def enableCollisionAvoidance(self, others, ellipsoidRadii):
    #     """Enables onboard collision avoidance.

    #     When enabled, avoids colliding with other Crazyflies by using the
    #     Buffered Voronoi Cells method [1]. Computation is performed onboard.

    #     Args:
    #         others (List[Crazyflie]): List of other :obj:`Crazyflie` objects.
    #             In simulation, collision avoidance is checked only with members
    #             of this list.  With real hardware, this list is **ignored**, and
    #             collision avoidance is checked with all other Crazyflies on the
    #             same radio channel.
    #         ellipsoidRadii (array-like of float[3]): Radii of collision volume ellipsoid in meters.
    #             The Crazyflie's boundary for collision checking is a tall
    #             ellipsoid. This accounts for the downwash effect: Due to the
    #             fast-moving stream of air produced by the rotors, the safe
    #             distance to pass underneath another rotorcraft is much further
    #             than the safe distance to pass to the side.

    #     [1] D. Zhou, Wang, Z., Bandyopadhyay, S., and Schwager, M.
    #         Fast, On-line Collision Avoidance for Dynamic Vehicles using
    #         Buffered Voronoi Cells.  IEEE Robotics and Automation Letters
    #         (RA-L), vol. 2, no. 2, pp. 1047 - 1054, 2017.
    #         https://msl.stanford.edu/fast-line-collision-avoidance-dynamic-vehicles-using-buffered-voronoi-cells
    #     """
    #     # Set radii before enabling to ensure collision avoidance never
    #     # observes a wrong radius value.
    #     self.setParams({
    #         "colAv/ellipsoidX": float(ellipsoidRadii[0]),
    #         "colAv/ellipsoidY": float(ellipsoidRadii[1]),
    #         "colAv/ellipsoidZ": float(ellipsoidRadii[2]),
    #     })
    #     self.setParam("colAv/enable", 1)


    # def disableCollisionAvoidance(self):
    #     """Disables onboard collision avoidance."""
    #     self.setParam("colAv/enable", 0)

    def emergency(self):
        """Emergency stop. Cuts power; causes future commands to be ignored.

        This command is useful if the operator determines that the control
        script is flawed, and that continuing to follow it will cause wrong/
        self-destructive behavior from the robots. In addition to cutting
        power to the motors, it ensures that any future commands, both high-
        level and streaming, will have no effect.

        The only ways to reset the firmware after an emergency stop has occurred
        are a physical hard reset or an nRF51 Reboot command.
        """
        req = Empty.Request()
        self.emergencyService.call_async(req)

    def takeoff(self, targetHeight, duration, groupMask = 0):
        """Execute a takeoff - fly straight up, then hover indefinitely.

        Asynchronous command; returns immediately.

        Args:
            targetHeight (float): The z-coordinate at which to hover.
            duration (float): How long until the height is reached. Seconds.
            groupMask (int): Group mask bits. See :meth:`setGroupMask()` doc.
        """
        req = Takeoff.Request()
        req.group_mask = groupMask
        req.height = targetHeight
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        self.takeoffService.call_async(req)

    def land(self, targetHeight, duration, groupMask = 0):
        """Execute a landing - fly straight down. User must cut power after.

        Asynchronous command; returns immediately.

        Args:
            targetHeight (float): The z-coordinate at which to land. Meters.
                Usually should be a few centimeters above the initial position
                to ensure that the controller does not try to penetrate the
                floor if the mocap coordinate origin is not perfect.
            duration (float): How long until the height is reached. Seconds.
            groupMask (int): Group mask bits. See :meth:`setGroupMask()` doc.
        """
        req = Land.Request()
        req.group_mask = groupMask
        req.height = targetHeight
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        self.landService.call_async(req)

    # def stop(self, groupMask = 0):
    #     """Cuts power to the motors when operating in low-level command mode.

    #     Intended for non-emergency scenarios, e.g. landing with the possibility
    #     of taking off again later. Future low- or high-level commands will
    #     restart the motors.

    #     Args:
    #         groupMask (int): Group mask bits. See :meth:`setGroupMask()` doc.
    #     """
    #     self.stopService(groupMask)

    def goTo(self, goal, yaw, duration, relative = False, groupMask = 0):
        """Move smoothly to the goal, then hover indefinitely.

        Asynchronous command; returns immediately.

        Plans a smooth trajectory from the current state to the goal position.
        Will stop smoothly at the goal with minimal overshoot. If the current
        state is at hover, the planned trajectory will be a straight line;
        however, if the current velocity is nonzero, the planned trajectory
        will be a smooth curve.

        Plans the trajectory by solving for the unique degree-7 polynomial that
        satisfies the initial conditions of the current position, velocity,
        and acceleration, and ends at the goal position with zero velocity and
        acceleration. The jerk (derivative of acceleration) is fixed at zero at
        both boundary conditions.

        Note: it is the user's responsibility to ensure that the goTo command
        is feasible. If the duration is too short, the trajectory will require
        impossible accelerations and velocities. The planner will not correct
        this, and the failure to achieve the desired states will cause the
        controller to become unstable.

        Args:
            goal (iterable of 3 floats): The goal position. Meters.
            yaw (float): The goal yaw angle (heading). Radians.
            duration (float): How long until the goal is reached. Seconds.
            relative (bool): If true, the goal position is interpreted as a
                relative offset from the current position. Otherwise, the goal
                position is interpreted as absolute coordintates in the global
                reference frame.
            groupMask (int): Group mask bits. See :meth:`setGroupMask()` doc.
        """
        req = GoTo.Request()
        req.group_mask = groupMask
        req.relative = relative
        req.goal = arrayToGeometryPoint(goal)
        req.yaw = float(yaw)
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        self.goToService.call_async(req)

    def uploadTrajectory(self, trajectoryId, pieceOffset, trajectory):
        """Uploads a piecewise polynomial trajectory for later execution.

        See uav_trajectory.py for more information about piecewise polynomial
        trajectories.

        Args:
            trajectoryId (int): ID number of this trajectory. Multiple
                trajectories can be uploaded. TODO: what is the maximum ID?
            pieceOffset (int): TODO(whoenig): explain this.
            trajectory (:obj:`pycrazyswarm.uav_trajectory.Trajectory`): Trajectory object.
        """
        pieces = []
        for poly in trajectory.polynomials:
            piece = TrajectoryPolynomialPiece()
            piece.duration = rclpy.duration.Duration(seconds=poly.duration).to_msg()
            piece.poly_x   = poly.px.p.tolist()
            piece.poly_y   = poly.py.p.tolist()
            piece.poly_z   = poly.pz.p.tolist()
            piece.poly_yaw = poly.pyaw.p.tolist()
            pieces.append(piece)
        req = UploadTrajectory.Request()
        req.trajectory_id = trajectoryId
        req.piece_offset = pieceOffset
        req.pieces = pieces
        self.uploadTrajectoryService.call_async(req)

    def startTrajectory(self, trajectoryId, timescale = 1.0, reverse = False, relative = True, groupMask = 0):
        """Begins executing a previously uploaded trajectory.

        Asynchronous command; returns immediately.

        Args:
            trajectoryId (int): ID number as given to :meth:`uploadTrajectory()`.
            timescale (float): Scales the trajectory duration by this factor.
                For example if timescale == 2.0, the trajectory will take twice
                as long to execute as the nominal duration.
            reverse (bool): If true, executes the trajectory backwards in time.
            relative (bool): If true (default), the position of the trajectory
                is shifted such that it begins at the current position setpoint.
                This is usually the desired behavior.
            groupMask (int): Group mask bits. See :meth:`setGroupMask()` doc.
        """
        req = StartTrajectory.Request()
        req.group_mask = groupMask
        req.trajectory_id = trajectoryId
        req.timescale = timescale
        req.reversed = reverse
        req.relative = relative
        self.startTrajectoryService.call_async(req)

    def notifySetpointsStop(self, remainValidMillisecs=100, groupMask=0):
        """Informs that streaming low-level setpoint packets are about to stop.

        Streaming setpoints are :meth:`cmdVelocityWorld`, :meth:`cmdFullState`,
        and so on. For safety purposes, they normally preempt onboard high-level
        commands such as :meth:`goTo`.

        Once preempted, the Crazyflie will not switch back to high-level
        commands (or other behaviors determined by onboard planning/logic) until
        a significant amount of time has elapsed where no low-level setpoint
        was received.

        This command short-circuits that waiting period to a user-chosen time.
        It should be called after sending the last low-level setpoint, and
        before sending any high-level command.

        A common use case is to execute the :meth:`land` command after using
        streaming setpoint modes.

        Args:
            remainValidMillisecs (int): Number of milliseconds that the last
                streaming setpoint should be followed before reverting to the
                onboard-determined behavior. May be longer e.g. if one radio
                is controlling many robots.
        """
        req = NotifySetpointsStop.Request()
        req.remain_valid_millisecs = remainValidMillisecs
        req.group_mask = groupMask
        self.notifySetpointsStopService.call_async(req)

    # def position(self):
    #     """Returns the last true position measurement from motion capture.

    #     If at least one position measurement for this robot has been received
    #     from the motion capture system since startup, this function returns
    #     immediately with the most recent measurement. However, if **no**
    #     position measurements have been received, it blocks until the first
    #     one arrives.

    #     Returns:
    #         position (np.array[3]): Current position. Meters.
    #     """
    #     self.tf.waitForTransform("/world", "/cf" + str(self.id), rospy.Time(0), rospy.Duration(10))
    #     position, quaternion = self.tf.lookupTransform("/world", "/cf" + str(self.id), rospy.Time(0))
    #     return np.array(position)

    # def getParam(self, name):
    #     """Returns the current value of the onboard named parameter.

    #     Parameters are named values of various primitive C types that control
    #     the firmware's behavior. For more information, see
    #     https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/logparam/.

    #     Parameters are read at system startup over the radio and cached.
    #     The ROS launch file can also be used to set parameter values at startup.
    #     Subsequent calls to :meth:`setParam()` will update the cached value.
    #     However, if the parameter changes for any other reason, the cached value
    #     might become stale. This situation is not common.

    #     Args:
    #         name (str): The parameter's name.

    #     Returns:
    #         value (Any): The parameter's value.
    #     """
    #     return rospy.get_param(self.prefix + "/" + name)

    def setParam(self, name, value):
        """Changes the value of the given parameter.

        See :meth:`getParam()` docs for overview of the parameter system.

        Args:
            name (str): The parameter's name.
            value (Any): The parameter's value.
        """
        param_name = self.prefix[1:] + ".params." + name
        param_type = self.paramTypeDict[name]
        if param_type == ParameterType.PARAMETER_INTEGER:
            param_value = ParameterValue(type=param_type, integer_value=int(value))
        elif param_type == ParameterType.PARAMETER_DOUBLE:
            param_value = ParameterValue(type=param_type, double_value=float(value))
        req = SetParameters.Request()
        req.parameters = [Parameter(name=param_name, value=param_value)]
        self.setParamsService.call_async(req)

    # def setParams(self, params):
    #     """Changes the value of several parameters at once.

    #     See :meth:`getParam()` docs for overview of the parameter system.

    #     Args:
    #         params (Dict[str, Any]): Dict of parameter names/values.
    #     """
    #     for name, value in params.items():
    #         rospy.set_param(self.prefix + "/" + name, value)
    #     self.updateParamsService(params.keys())

    def cmdFullState(self, pos, vel, acc, yaw, omega):
        """Sends a streaming full-state controller setpoint command.

        The full-state setpoint is most useful for aggressive maneuvers where
        feedforward inputs for acceleration and angular velocity are critical
        to obtaining good tracking performance. Full-state setpoints can be
        obtained from any trajectory parameterization that is at least three
        times differentiable, e.g. piecewise polynomials.

        Sending a streaming setpoint of any type will force a change from
        high-level to low-level command mode. Currently, there is no mechanism
        to change back, but it is a high-priority feature to implement.
        This means it is not possible to use e.g. :meth:`land()` or
        :meth:`goTo()` after a streaming setpoint has been sent.

        Args:
            pos (array-like of float[3]): Position. Meters.
            vel (array-like of float[3]): Velocity. Meters / second.
            acc (array-like of float[3]): Acceleration. Meters / second^2.
            yaw (float): Yaw angle. Radians.
            omega (array-like of float[3]): Angular velocity in body frame.
                Radians / sec.
        """
        self.cmdFullStateMsg.header.stamp = self.node.get_clock().now().to_msg()
        self.cmdFullStateMsg.pose.position.x    = pos[0]
        self.cmdFullStateMsg.pose.position.y    = pos[1]
        self.cmdFullStateMsg.pose.position.z    = pos[2]
        self.cmdFullStateMsg.twist.linear.x     = vel[0]
        self.cmdFullStateMsg.twist.linear.y     = vel[1]
        self.cmdFullStateMsg.twist.linear.z     = vel[2]
        self.cmdFullStateMsg.acc.x              = acc[0]
        self.cmdFullStateMsg.acc.y              = acc[1]
        self.cmdFullStateMsg.acc.z              = acc[2]
        q = rowan.from_euler(0, 0, yaw)
        self.cmdFullStateMsg.pose.orientation.w = q[0]
        self.cmdFullStateMsg.pose.orientation.x = q[1]
        self.cmdFullStateMsg.pose.orientation.y = q[2]
        self.cmdFullStateMsg.pose.orientation.z = q[3]
        self.cmdFullStateMsg.twist.angular.x    = omega[0]
        self.cmdFullStateMsg.twist.angular.y    = omega[1]
        self.cmdFullStateMsg.twist.angular.z    = omega[2]
        self.cmdFullStatePublisher.publish(self.cmdFullStateMsg)

    # def cmdVelocityWorld(self, vel, yawRate):
    #     """Sends a streaming velocity-world controller setpoint command.

    #     In this mode, the PC specifies desired velocity vector and yaw rate.
    #     The onboard controller will try to achive this velocity.

    #     NOTE: the Mellinger controller is Crazyswarm's default controller, but
    #     it has not been tuned (or even tested) for velocity control mode.
    #     Switch to the PID controller by changing
    #     `firmwareParams.stabilizer.controller` to `1` in your launch file.

    #     Sending a streaming setpoint of any type will force a change from
    #     high-level to low-level command mode. Currently, there is no mechanism
    #     to change back, but it is a high-priority feature to implement.
    #     This means it is not possible to use e.g. :meth:`land()` or
    #     :meth:`goTo()` after a streaming setpoint has been sent.

    #     Args:
    #         vel (array-like of float[3]): Velocity. Meters / second.
    #         yawRate (float): Yaw angular velocity. Degrees / second.
    #     """
    #     self.cmdVelocityWorldMsg.header.stamp = rospy.Time.now()
    #     self.cmdVelocityWorldMsg.header.seq += 1
    #     self.cmdVelocityWorldMsg.vel.x = vel[0]
    #     self.cmdVelocityWorldMsg.vel.y = vel[1]
    #     self.cmdVelocityWorldMsg.vel.z = vel[2]
    #     self.cmdVelocityWorldMsg.yawRate = yawRate
    #     self.cmdVelocityWorldPublisher.publish(self.cmdVelocityWorldMsg)

    # def cmdStop(self):
    #     """Interrupts any high-level command to stop and cut motor power.

    #     Intended for non-emergency scenarios, e.g. landing with the possibility
    #     of taking off again later. Future low- or high-level commands will
    #     restart the motors. Equivalent of :meth:`stop()` when in high-level mode.
    #     """
    #     self.cmdStopPublisher.publish(std_msgs.msg.Empty())

    # def cmdVel(self, roll, pitch, yawrate, thrust):
    #     """Sends a streaming command of the "easy mode" manual control inputs.

    #     The (absolute roll & pitch, yaw rate, thrust) inputs are typically
    #     used for manual flying by beginners or causal pilots, as opposed to the
    #     "acrobatic mode" inputs where roll and pitch rates are controlled
    #     instead of absolute angles. This mode limits the possible maneuvers,
    #     e.g. it is not possible to do a flip because the controller joystick
    #     would need to rotate 360 degrees.

    #     For more information on streaming setpoint commands, see the
    #     :meth:`cmdFullState()` documentation.

    #     !NOTE!: The angles and angular velocities in this command are in
    #     degrees, whereas they are in radians for cmdFullState.

    #     TODO: should we change the name from cmdVel to something else?
    #     IMO (japreiss), cmdVel implies controlling linear velocity.

    #     Args:
    #         roll (float): Roll angle. Degrees. Positive values == roll right.
    #         pitch (float): Pitch angle. Degrees. Positive values == pitch
    #             forward/down.
    #         yawrate (float): Yaw angular velocity. Degrees / second. Positive
    #             values == turn counterclockwise.
    #         thrust (float): Thrust magnitude. Non-meaningful units in [0, 2^16),
    #             where the maximum value corresponds to maximum thrust.
    #     """
    #     msg = geometry_msgs.msg.Twist()
    #     msg.linear.x = pitch
    #     msg.linear.y = roll
    #     msg.angular.z = yawrate
    #     msg.linear.z = thrust
    #     self.cmdVelPublisher.publish(msg)

    def cmdPosition(self, pos, yaw = 0.):
        """Sends a streaming command of absolute position and yaw setpoint.

        Useful for slow maneuvers where a high-level planner determines the
        desired position, and the rest is left to the onboard controller.

        For more information on streaming setpoint commands, see the
        :meth:`cmdFullState()` documentation.
        Args:
            pos (array-like of float[3]): Position. Meters.
            yaw (float): Yaw angle. Radians.
        """
        self.cmdPositionMsg.header.stamp = self.node.get_clock().now().to_msg()
        self.cmdPositionMsg.x   = pos[0]
        self.cmdPositionMsg.y   = pos[1]
        self.cmdPositionMsg.z   = pos[2]
        self.cmdPositionMsg.yaw = yaw
        self.cmdPositionPublisher.publish(self.cmdPositionMsg)

    # def setLEDColor(self, r, g, b):
    #     """Sets the color of the LED ring deck.

    #     While most params (such as PID gains) only need to be set once, it is
    #     common to change the LED ring color many times during a flight, e.g.
    #     as some kind of status indicator. This method makes it convenient.

    #     PRECONDITION: The param "ring/effect" must be set to 7 (solid color)
    #     for this command to have any effect. The default mode uses the ring
    #     color to indicate radio connection quality.

    #     This is a blocking command, so it may cause stability problems for
    #     large swarms and/or high-frequency changes.

    #     Args:
    #         r (float): Red component of color, in range [0, 1].
    #         g (float): Green component of color, in range [0, 1].
    #         b (float): Blue component of color, in range [0, 1].
    #     """
    #     self.setParam("ring/solidRed", int(r * 255))
    #     self.setParam("ring/solidGreen", int(g * 255))
    #     self.setParam("ring/solidBlue", int(b * 255))


class CrazyflieServer(rclpy.node.Node):
    """Object for broadcasting commands to all robots at once.

    Also is the container for the individual :obj:`Crazyflie` objects.

    Attributes:
        crazyfiles (List[Crazyflie]): List of one Crazyflie object per robot,
            as determined by the crazyflies.yaml config file.
        crazyfliesById (Dict[int, Crazyflie]): Index to the same Crazyflie
            objects by their ID number (last byte of radio address).
    """
    def __init__(self):
        """Initialize the server. Waits for all ROS services before returning.
        """
        super().__init__("CrazyflieAPI")
        self.emergencyService = self.create_client(Empty, "all/emergency")
        self.emergencyService.wait_for_service()

        self.takeoffService = self.create_client(Takeoff, "all/takeoff")
        self.takeoffService.wait_for_service()
        self.landService = self.create_client(Land, "all/land")
        self.landService.wait_for_service()
        self.goToService = self.create_client(GoTo, "all/go_to")
        self.goToService.wait_for_service()
        self.startTrajectoryService = self.create_client(StartTrajectory, "all/start_trajectory")
        self.startTrajectoryService.wait_for_service()
        self.setParamsService = self.create_client(SetParameters, "/crazyflie_server/set_parameters")
        self.setParamsService.wait_for_service()

        self.cmdFullStatePublisher = self.create_publisher(FullState, "all/cmd_full_state", 1)
        self.cmdFullStateMsg = FullState()
        self.cmdFullStateMsg.header.frame_id = "/world"

        cfnames = []
        for srv_name, srv_types in self.get_service_names_and_types():
            if "crazyflie_interfaces/srv/StartTrajectory" in srv_types:
                # remove "/" and "/start_trajectory"
                cfname = srv_name[1:-17]
                if cfname != "all":
                    cfnames.append(cfname)

        # Query all parameters
        listParamsService = self.create_client(ListParameters, "/crazyflie_server/list_parameters")
        listParamsService.wait_for_service()
        req = ListParameters.Request()
        req.depth = ListParameters.Request.DEPTH_RECURSIVE
        req.prefixes = []
        future = listParamsService.call_async(req)
        params = []
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                # Filter the parameters that belong to this Crazyflie
                response = future.result()
                for p in response.result.names:
                    if ".params." in p:
                        params.append(p)
                break

        # Find the types for the parameters and store them
        describeParametersService = self.create_client(DescribeParameters, "/crazyflie_server/describe_parameters")
        describeParametersService.wait_for_service()
        req = DescribeParameters.Request()
        req.names = params
        future = describeParametersService.call_async(req)
        allParamTypeDicts = defaultdict(dict)
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                # Filter the parameters that belong to this Crazyflie
                response = future.result()
                for p, d in zip(params, response.descriptors):
                    idx = p.index(".params.")
                    cf_name = p[0:idx]
                    param_name = p[idx+8:]
                    t = d.type
                    if cf_name in allParamTypeDicts:
                        allParamTypeDicts[cf_name][param_name] = t
                    else:
                        allParamTypeDicts[cf_name] = {param_name: t}
                break
        self.paramTypeDict = allParamTypeDicts["all"]

        self.crazyflies = []
        self.crazyfliesById = dict()
        self.crazyfliesByName = dict()
        for cfname in cfnames:
            cf = Crazyflie(self, cfname, allParamTypeDicts[cfname])
            self.crazyflies.append(cf)
            self.crazyfliesByName[cfname] = cf
            # For legacy crazyswarm1 code, also provide crazyfliesById
            cfid = int(cf.uri[-2:], 16)
            self.crazyfliesById[cfid] = cf

    def emergency(self):
        """Emergency stop. Cuts power; causes future commands to be ignored.

        This command is useful if the operator determines that the control
        script is flawed, and that continuing to follow it will cause wrong/
        self-destructive behavior from the robots. In addition to cutting
        power to the motors, it ensures that any future commands, both high-
        level and streaming, will have no effect.

        The only ways to reset the firmware after an emergency stop has occurred
        are a physical hard reset or an nRF51 Reboot command.
        """
        req = Empty.Request()
        self.emergencyService.call_async(req)

    def takeoff(self, targetHeight, duration, groupMask = 0):
        """Broadcasted takeoff - fly straight up, then hover indefinitely.

        Broadcast version of :meth:`Crazyflie.takeoff()`. All robots that match the
        groupMask take off at exactly the same time. Use for synchronized
        movement. Asynchronous command; returns immediately.

        Args:
            targetHeight (float): The z-coordinate at which to hover.
            duration (float): How long until the height is reached. Seconds.
            groupMask (int): Group mask bits. See :meth:`setGroupMask()` doc.
        """
        req = Takeoff.Request()
        req.group_mask = groupMask
        req.height = targetHeight
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        self.takeoffService.call_async(req)

    def land(self, targetHeight, duration, groupMask = 0):
        """Broadcasted landing - fly straight down. User must cut power after.

        Broadcast version of :meth:`Crazyflie.land()`. All robots that match the
        groupMask land at exactly the same time. Use for synchronized
        movement. Asynchronous command; returns immediately.

        Args:
            targetHeight (float): The z-coordinate at which to land. Meters.
                Usually should be a few centimeters above the initial position
                to ensure that the controller does not try to penetrate the
                floor if the mocap coordinate origin is not perfect.
            duration (float): How long until the height is reached. Seconds.
            groupMask (int): Group mask bits. See :meth:`Crazyflie.setGroupMask()` doc.
        """
        req = Land.Request()
        req.group_mask = groupMask
        req.height = targetHeight
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        self.landService.call_async(req)

    def goTo(self, goal, yaw, duration, groupMask = 0):
        """Broadcasted goTo - Move smoothly to goal, then hover indefinitely.

        Broadcast version of :meth:`Crazyflie.goTo()`. All robots that match the
        groupMask start moving at exactly the same time. Use for synchronized
        movement. Asynchronous command; returns immediately.

        While the individual goTo() supports both relative and absolute
        coordinates, the broadcasted goTo only makes sense with relative
        coordinates (since absolute broadcasted goTo() would cause a collision).
        Therefore, there is no `relative` kwarg.

        See docstring of :meth:`Crazyflie.goTo()` for additional details.

        Args:
            goal (iterable of 3 floats): The goal offset. Meters.
            yaw (float): The goal yaw angle (heading). Radians.
            duration (float): How long until the goal is reached. Seconds.
            groupMask (int): Group mask bits. See :meth:`Crazyflie.setGroupMask()` doc.
        """
        req = GoTo.Request()
        req.group_mask = groupMask
        req.relative = True
        req.goal = arrayToGeometryPoint(goal)
        req.yaw = yaw
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        self.goToService.call_async(req)

    def startTrajectory(self, trajectoryId, timescale = 1.0, reverse = False, relative = True, groupMask = 0):
        """Broadcasted - begins executing a previously uploaded trajectory.

        Broadcast version of :meth:`Crazyflie.startTrajectory()`.
        Asynchronous command; returns immediately.

        Args:
            trajectoryId (int): ID number as given to :meth:`Crazyflie.uploadTrajectory()`.
            timescale (float): Scales the trajectory duration by this factor.
                For example if timescale == 2.0, the trajectory will take twice
                as long to execute as the nominal duration.
            reverse (bool): If true, executes the trajectory backwards in time.
            relative (bool): If true (default), the position of the trajectory
                is shifted such that it begins at the current position setpoint.
            groupMask (int): Group mask bits. See :meth:`Crazyflie.setGroupMask()` doc.
        """
        req = StartTrajectory.Request()
        req.group_mask = groupMask
        req.trajectory_id = trajectoryId
        req.timescale = timescale
        req.reversed = reverse
        req.relative = relative
        self.startTrajectoryService.call_async(req)

    def setParam(self, name, value):
        """Broadcasted setParam. See Crazyflie.setParam() for details."""
        param_name = "all.params." + name
        param_type = self.paramTypeDict[name]
        if param_type == ParameterType.PARAMETER_INTEGER:
            param_value = ParameterValue(type=param_type, integer_value=int(value))
        elif param_type == ParameterType.PARAMETER_DOUBLE:
            param_value = ParameterValue(type=param_type, double_value=float(value))
        req = SetParameters.Request()
        req.parameters = [Parameter(name=param_name, value=param_value)]
        self.setParamsService.call_async(req)

    def cmdFullState(self, pos, vel, acc, yaw, omega):
        """Sends a streaming full-state controller setpoint command.

        The full-state setpoint is most useful for aggressive maneuvers where
        feedforward inputs for acceleration and angular velocity are critical
        to obtaining good tracking performance. Full-state setpoints can be
        obtained from any trajectory parameterization that is at least three
        times differentiable, e.g. piecewise polynomials.

        Sending a streaming setpoint of any type will force a change from
        high-level to low-level command mode. Currently, there is no mechanism
        to change back, but it is a high-priority feature to implement.
        This means it is not possible to use e.g. :meth:`land()` or
        :meth:`goTo()` after a streaming setpoint has been sent.

        Args:
            pos (array-like of float[3]): Position. Meters.
            vel (array-like of float[3]): Velocity. Meters / second.
            acc (array-like of float[3]): Acceleration. Meters / second^2.
            yaw (float): Yaw angle. Radians.
            omega (array-like of float[3]): Angular velocity in body frame.
                Radians / sec.
        """
        self.cmdFullStateMsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdFullStateMsg.pose.position.x    = pos[0]
        self.cmdFullStateMsg.pose.position.y    = pos[1]
        self.cmdFullStateMsg.pose.position.z    = pos[2]
        self.cmdFullStateMsg.twist.linear.x     = vel[0]
        self.cmdFullStateMsg.twist.linear.y     = vel[1]
        self.cmdFullStateMsg.twist.linear.z     = vel[2]
        self.cmdFullStateMsg.acc.x              = acc[0]
        self.cmdFullStateMsg.acc.y              = acc[1]
        self.cmdFullStateMsg.acc.z              = acc[2]
        q = rowan.from_euler(0, 0, yaw)
        self.cmdFullStateMsg.pose.orientation.w = q[0]
        self.cmdFullStateMsg.pose.orientation.x = q[1]
        self.cmdFullStateMsg.pose.orientation.y = q[2]
        self.cmdFullStateMsg.pose.orientation.z = q[3]
        self.cmdFullStateMsg.twist.angular.x    = omega[0]
        self.cmdFullStateMsg.twist.angular.y    = omega[1]
        self.cmdFullStateMsg.twist.angular.z    = omega[2]
        self.cmdFullStatePublisher.publish(self.cmdFullStateMsg)