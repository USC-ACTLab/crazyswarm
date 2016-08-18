# -*- coding: utf-8 -*-
#     ||
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2011-2013 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
"""
Linux joystick driver using the Linux input_joystick subsystem. Requires sysfs
to be mounted on /sys and /dev/input/js* to be readable.

This module is very linux specific but should work on any CPU platform
"""
import ctypes
import glob
import logging
import os
import struct
import sys

if not sys.platform.startswith('linux'):
    raise Exception("Only supported on Linux")

try:
    import fcntl
except ImportError as e:
    raise Exception("fcntl library probably not installed ({})".format(e))

__author__ = 'Bitcraze AB'
__all__ = ['Joystick']

logger = logging.getLogger(__name__)

JS_EVENT_FMT = "@IhBB"
JE_TIME = 0
JE_VALUE = 1
JE_TYPE = 2
JE_NUMBER = 3

JS_EVENT_BUTTON = 0x001
JS_EVENT_AXIS = 0x002
JS_EVENT_INIT = 0x080

# ioctls
JSIOCGAXES = 0x80016a11
JSIOCGBUTTONS = 0x80016a12

MODULE_MAIN = "Joystick"
MODULE_NAME = "linuxjsdev"


class JEvent(object):
    """
    Joystick event class. Encapsulate single joystick event.
    """

    def __init__(self, evt_type, number, value):
        self.type = evt_type
        self.number = number
        self.value = value

    def __repr__(self):
        return "JEvent(type={}, number={}, value={})".format(
            self.type, self.number, self.value)

# Constants
TYPE_BUTTON = 1
TYPE_AXIS = 2


class _JS():

    def __init__(self, num, name):
        self.num = num
        self.name = name
        self._f_name = "/dev/input/js{}".format(num)
        self._f = None

        self.opened = False
        self.buttons = []
        self.axes = []
        self._prev_pressed = {}

    def open(self):
        if self._f:
            raise Exception("{} at {} is already "
                            "opened".format(self.name, self._f_name))

        self._f = open("/dev/input/js{}".format(self.num), "rb")
        fcntl.fcntl(self._f.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)

        # Get number of axis and button
        val = ctypes.c_int()
        if fcntl.ioctl(self._f.fileno(), JSIOCGAXES, val) != 0:
            self._f.close()
            self._f = None
            raise Exception("Failed to read number of axes")

        self.axes = list(0 for i in range(val.value))
        if fcntl.ioctl(self._f.fileno(), JSIOCGBUTTONS, val) != 0:
            self._f.close()
            self._f = None
            raise Exception("Failed to read number of axes")

        self.buttons = list(0 for i in range(val.value))
        self.__initvalues()

    def close(self):
        """Open the joystick device"""
        if not self._f:
            return

        logger.info("Closed {} ({})".format(self.name, self.num))

        self._f.close()
        self._f = None

    def __initvalues(self):
        """Read the buttons and axes initial values from the js device"""
        for _ in range(len(self.axes) + len(self.buttons)):
            data = self._f.read(struct.calcsize(JS_EVENT_FMT))
            jsdata = struct.unpack(JS_EVENT_FMT, data)
            self.__updatestate(jsdata)

    def __updatestate(self, jsdata):
        """Update the internal absolute state of buttons and axes"""
        if jsdata[JE_TYPE] & JS_EVENT_AXIS != 0:
            self.axes[jsdata[JE_NUMBER]] = jsdata[JE_VALUE] / 32768.0
        elif jsdata[JE_TYPE] & JS_EVENT_BUTTON != 0:
            self.buttons[jsdata[JE_NUMBER]] = jsdata[JE_VALUE]

    def __decode_event(self, jsdata):
        """ Decode a jsdev event into a dict """
        # TODO: Add timestamp?
        if jsdata[JE_TYPE] & JS_EVENT_AXIS != 0:
            return JEvent(evt_type=TYPE_AXIS,
                          number=jsdata[JE_NUMBER],
                          value=jsdata[JE_VALUE] / 32768.0)
        if jsdata[JE_TYPE] & JS_EVENT_BUTTON != 0:
            return JEvent(evt_type=TYPE_BUTTON,
                          number=jsdata[JE_NUMBER],
                          value=jsdata[JE_VALUE] / 32768.0)

    def _read_all_events(self):
        """Consume all the events queued up in the JS device"""
        try:
            while True:
                data = self._f.read(struct.calcsize(JS_EVENT_FMT))
                jsdata = struct.unpack(JS_EVENT_FMT, data)
                self.__updatestate(jsdata)
        except IOError as e:
            if e.errno != 11:
                logger.info(str(e))
                self._f.close()
                self._f = None
                raise IOError("Device has been disconnected")
        except TypeError:
            pass
        except ValueError:
            # This will happen if I/O operations are done on a closed device,
            # which is the case when you first close and then open the device
            # while switching device. But, in order for SDL2 to work on Linux
            # (for debugging) the device needs to be closed before it's opened.
            # This is the workaround to make both cases work.
            pass

    def read(self):
        """ Returns a list of all joystick event since the last call """
        if not self._f:
            raise Exception("Joystick device not opened")

        self._read_all_events()

        return [self.axes, self.buttons]


class Joystick():
    """
    Linux jsdev implementation of the Joystick class
    """

    def __init__(self):
        self.name = MODULE_NAME
        self._js = {}
        self._devices = []

    def devices(self):
        """
        Returns a dict with device_id as key and device name as value of all
        the detected devices (result is cached once one or more device are
        found).
        """

        if len(self._devices) == 0:
            syspaths = glob.glob("/sys/class/input/js*")

            for path in syspaths:
                device_id = int(os.path.basename(path)[2:])
                with open(path + "/device/name") as namefile:
                    name = namefile.read().strip()
                self._js[device_id] = _JS(device_id, name)
                self._devices.append({"id": device_id, "name": name})

        return self._devices

    def open(self, device_id):
        """
        Open the joystick device. The device_id is given by available_devices
        """
        self._js[device_id].open()

    def close(self, device_id):
        """Open the joystick device"""
        self._js[device_id].close()

    def read(self, device_id):
        """ Returns a list of all joystick event since the last call """
        return self._js[device_id].read()
