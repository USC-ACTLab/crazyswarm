#!/usr/bin/env python

import Tkinter
import yaml
import os
import subprocess
import re
import time
import threading
import math
import numpy as np
import sys, tty, termios
import ttk
from pycrazyswarm import *

os.chdir(os.path.dirname(os.path.realpath(__file__)))
SCRIPTDIR = "../../../../scripts/"
Z = 0.6

class CFWidget(Tkinter.Frame):
    def __init__(self, parent, name):
        Tkinter.Frame.__init__(self, parent)
        self.checked = Tkinter.BooleanVar()
        checkbox = Tkinter.Checkbutton(self, variable=self.checked, command=save,
                                       padx=0, pady=0)
        checkbox.grid(row=0, column=0, sticky='E')
        nameLabel = Tkinter.Label(self, text=name, padx=0, pady=0)
        nameLabel.grid(row=0, column=1, sticky='W')
        self.batteryLabel = Tkinter.Label(self, text="", fg="#999999", padx=0, pady=0)
        self.batteryLabel.grid(row=1, column=0, columnspan=2, sticky='E')
        self.versionLabel = Tkinter.Label(self, text="", fg="#999999", padx=0, pady=0)
        self.versionLabel.grid(row=2, column=0, columnspan=2, sticky='E')

# read a yaml file
def read_by_id(path):
    by_id = {}
    with open(path, 'r') as ymlfile:
        root = yaml.load(ymlfile)
        for node in root["crazyflies"]:
            id = int(node["id"])
            by_id[id] = node
    return by_id


def save():
    nodes = [node for id, node in allCrazyflies.items() if widgets[id].checked.get()]
    with open("../launch/crazyflies.yaml", 'w') as outfile:
        yaml.dump({"crazyflies": nodes}, outfile)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def minmax(a, b):
    return min(a, b), max(a, b)


def mouseDown(event):
    global drag_start, drag_startstate
    drag_start = (event.x_root, event.y_root)
    drag_startstate = [cf.checked.get() for cf in widgets.values()]


def mouseUp(event):
    save()


def drag(event, select):
    x, y = event.x_root, event.y_root
    dragx0, dragx1 = minmax(drag_start[0], x)
    dragy0, dragy1 = minmax(drag_start[1], y)

    def dragcontains(widget):
        x0 = widget.winfo_rootx()
        y0 = widget.winfo_rooty()
        x1 = x0 + widget.winfo_width()
        y1 = y0 + widget.winfo_height()
        return not (x0 > dragx1 or x1 < dragx0 or y0 > dragy1 or y1 < dragy0)

    # depending on interation over dicts being consistent
    for initial, cf in zip(drag_startstate, widgets.values()):
        if dragcontains(cf):
            cf.checked.set(select)
        else:
            cf.checked.set(initial)


# buttons for clearing/filling all checkboxes
def clear():
    for box in widgets.values():
        box.checked.set(False)
    save()


def fill():
    for box in widgets.values():
        box.checked.set(True)
    save()


def takeoff():
    for cf in cfs:
        cf.takeoff(targetHeight=Z, duration=1.0 + Z)
    timeHelper.sleep(1.0 + Z)
    print "Take off"


def startPlanning():
    global planning_started
    planning_started = True
    allcfs.startPlanning()
    print "Start planning"


def startPatrol():
    global planning_started
    planning_started = True
    allcfs.startPatrol()
    print "Start patrol"


def back():
    global planning_started
    planning_started = True
    allcfs.stopPatrol()
    print "Go back to start position"


def land():
    global planning_started
    if planning_started:
        planning_started = False
        allcfs.stopPlanning()
    else:
        for cf in cfs:
            cf.land(targetHeight=0.02, duration=1.0 + Z)
    print "Land"


def emergencyStop():
    global planning_started
    planning_started = False
    allcfs.emergency()
    print "Emergency stop"


def setMission():
    name = combo.get() + ".json"
    print "Set mission: " + name
    command = 'rosservice call /change_mission "' + name + '"'
    os.system(command)


def mkbutton(parent, name, command):
    button = Tkinter.Button(parent, text=name, command=command)
    button.pack(side='left')


def handleKey(event):
    global planning_started, get_key_input
    if not get_key_input:
        return

    k = event.char
    if k == 'w':
        for cf in cfs:
            cf.goTo(np.array([0.2, 0, 0]), 0, 0.05, relative=True)
        print "front"
    elif k == 's':
        for cf in cfs:
            cf.goTo(np.array([-0.2, 0, 0]), 0, 0.05, relative=True)
        print "back"
    elif k == 'd':
        for cf in cfs:
            cf.goTo(np.array([0, -0.2, 0]), 0, 0.05, relative=True)
        print "right"
    elif k == 'a':
        for cf in cfs:
            cf.goTo(np.array([0, 0.2, 0]), 0, 0.05, relative=True)
        print "left"
    elif k == 'z':
        for cf in cfs:
            cf.goTo(np.array([0, 0, 0.1]), 0, 0.03, relative=True)
        print "up"
    elif k == 'c':
        for cf in cfs:
            cf.goTo(np.array([0, 0, -0.1]), 0, 0.03, relative=True)
        print "down"
    elif k == 'q':
        for cf in cfs:
            cf.goTo(np.array([0, 0, 0]), math.pi / 8, 0.1, relative=True)
        print "turn left"
    elif k == 'e':
        for cf in cfs:
            cf.goTo(np.array([0, 0, 0]), -math.pi / 8, 0.1, relative=True)
        print "turn right"
    elif k == 'k':
        emergencyStop()
    elif k == 't':
        takeoff()
    elif k == 'l':
        land()
    elif k == 'p':
        startPlanning()
    elif k == 'o':
        startPatrol()
    elif k == 'b':
        back()
    else:
        print "Invalid key"


if __name__ == '__main__':
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allCrazyflies = read_by_id("../launch/allCrazyflies.yaml")
    enabled = read_by_id("../launch/crazyflies.yaml").keys()
    with open("../launch/crazyflieTypes.yaml", 'r') as ymlfile:
        data = yaml.load(ymlfile)
    cfTypes = data["crazyflieTypes"]

    ids = read_by_id("../launch/crazyflies.yaml").keys()
    cfs = [allcfs.crazyfliesById[i] for i in ids]

    # compute absolute pixel coordinates from the initial positions
    positions = [node["initialPosition"] for node in allCrazyflies.values()]
    DOWN_DIR = [-1, 0]
    RIGHT_DIR = [0, -1]
    pixel_x = [60 * dot(pos, RIGHT_DIR) for pos in positions]
    pixel_y = [60 * dot(pos, DOWN_DIR) for pos in positions]
    xmin, ymin = min(pixel_x), min(pixel_y)
    xmax, ymax = max(pixel_x), max(pixel_y)

    # construct the main window
    top = Tkinter.Tk()
    top.title('Mavswarm Manager')

    # construct the frame containing the absolute-positioned checkboxes
    width = xmax - xmin + 50  # account for checkbox + text width
    height = ymax - ymin + 50  # account for checkbox + text height
    frame = Tkinter.Frame(top, width=width, height=height)

    # construct all the checkboxes
    widgets = {}
    for (id, node), x, y in zip(allCrazyflies.items(), pixel_x, pixel_y):
        w = CFWidget(frame, str(id))
        w.place(x=x - xmin, y=y - ymin)
        w.checked.set(id in enabled)
        widgets[id] = w

    # dragging functionality - TODO alt-drag to deselect
    drag_start = None
    drag_startstate = None

    # flags
    planning_started = False
    get_key_input = False

    top.bind('<ButtonPress-1>', mouseDown)
    top.bind('<ButtonPress-3>', mouseDown)
    top.bind('<B1-Motion>', lambda event: drag(event, True))
    top.bind('<B3-Motion>', lambda event: drag(event, False))
    top.bind('<ButtonRelease-1>', mouseUp)
    top.bind('<ButtonRelease-3>', mouseUp)
    top.bind('<Key>', handleKey)

    buttons = Tkinter.Frame(top)
    mkbutton(buttons, "Clear", clear)
    mkbutton(buttons, "Fill", fill)

    scriptButtons = Tkinter.Frame(top)
    mkbutton(scriptButtons, "Takeoff", takeoff)
    mkbutton(scriptButtons, "Key", emergencyStop)
    mkbutton(scriptButtons, "Start", startPlanning)
    mkbutton(scriptButtons, "Patrol", startPatrol)
    mkbutton(scriptButtons, "Back", back)
    mkbutton(scriptButtons, "Land", land)
    mkbutton(scriptButtons, "Kill", emergencyStop)

    missionButtons = Tkinter.Frame(top)
    combo = ttk.Combobox(missionButtons, values=["maze1", "maze1_2", "ccc", "ddd"])
    combo.pack(side='left')
    combo.current(0)
    mkbutton(missionButtons, "Set mission", setMission)

    # currently not supported
    # mkbutton(scriptButtons, "version", checkVersion)
    # mkbutton(scriptButtons, "sysOff", sysOff)
    # mkbutton(scriptButtons, "reboot", reboot)
    # mkbutton(scriptButtons, "flash (STM)", flashSTM)
    # mkbutton(scriptButtons, "flash (NRF)", flashNRF)

    buttons.pack()
    frame.pack(padx=10, pady=10)
    scriptButtons.pack()
    missionButtons.pack()
    top.mainloop()
