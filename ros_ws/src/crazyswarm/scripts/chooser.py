import Tkinter
import yaml
import os
import subprocess
import re
import time
import threading

os.chdir(os.path.dirname(os.path.realpath(__file__)))
SCRIPTDIR = "../../../../scripts/"

# read a yaml file
def read_by_id(path):
	by_id = {}
	with open(path, 'r') as ymlfile:
		root = yaml.load(ymlfile)
		for node in root["crazyflies"]:
			id = int(node["id"])
			by_id[id] = node
	return by_id

all49 = read_by_id("../launch/all49.yaml")
assert(len(all49) == 49)
enabled = read_by_id("../launch/crazyflies.yaml").keys()

# compute absolute pixel coordinates from the initial positions
positions = [node["initialPosition"] for node in all49.values()]
DOWN_DIR = [-1, 0]
RIGHT_DIR = [0, -1]
def dot(a, b):
	return a[0] * b[0] + a[1] * b[1]
pixel_x = [100 * dot(pos, RIGHT_DIR) for pos in positions]
pixel_y = [100 * dot(pos, DOWN_DIR) for pos in positions]
xmin, ymin = min(pixel_x), min(pixel_y)
xmax, ymax = max(pixel_x), max(pixel_y)

# construct the main window
top = Tkinter.Tk()
top.title('Crazyflie Chooser')

# construct the frame containing the absolute-positioned checkboxes
width = xmax - xmin + 50 # account for checkbox + text width
height = ymax - ymin + 30 # account for checkbox + text height
frame = Tkinter.Frame(top, width=width, height=height)

# construct all the checkboxes
toggles = {} # map crazyflie id to Tkinter data-binding variable
widgets = {}
for (id, node), x, y in zip(all49.items(), pixel_x, pixel_y):
	toggles[id] = Tkinter.BooleanVar()
	toggles[id].set(id in enabled)
	checkbox = Tkinter.Checkbutton(frame, text=str(id),
		justify='left', variable=toggles[id])
	checkbox.place(x = x - xmin, y = y - ymin)
	widgets[id] = checkbox

defaultBackground = widgets.values()[0].cget('background')

# dragging functionality - TODO alt-drag to deselect
drag_start = None
drag_startstate = None

def minmax(a, b):
	return min(a, b), max(a, b)

def mouseDown(event):
	global drag_start, drag_startstate
	drag_start = (event.x_root, event.y_root)
	drag_startstate = [toggle.get() for toggle in toggles.values()]

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
	for initial, toggle, checkbox in zip(drag_startstate, toggles.values(), widgets.values()):
		if dragcontains(checkbox):
			toggle.set(select)
		else:
			toggle.set(initial)

top.bind('<ButtonPress-1>', mouseDown)
top.bind('<ButtonPress-3>', mouseDown)
top.bind('<B1-Motion>', lambda event: drag(event, True))
top.bind('<B3-Motion>', lambda event: drag(event, False))

# construct top buttons for yaml configuration
def save():
	nodes = [node for id, node in all49.items() if toggles[id].get()]
	with open("../launch/crazyflies.yaml", 'w') as outfile:
		yaml.dump({"crazyflies": nodes}, outfile)

def clear():
	for box in toggles.values():
		box.set(False)

def fill():
	for box in toggles.values():
		box.set(True)

buttons = Tkinter.Frame(top)
saveButton = Tkinter.Button(buttons, text="Save", command=save)
saveButton.pack(side='left')
clearButton = Tkinter.Button(buttons, text="Clear", command=clear)
clearButton.pack(side='left')
fillButton = Tkinter.Button(buttons, text="Fill", command=fill)
fillButton.pack(side='left')

# construct bottom buttons for utility scripts
def sysOff():
	subprocess.Popen(["python3", SCRIPTDIR + "sysOff.py"])
def reboot():
	subprocess.Popen(["python3", SCRIPTDIR + "rebootAll.py"])
def flash():
	subprocess.Popen(["python3", SCRIPTDIR + "flashAll.py", "-stm32"])

import random
def checkBattery():
	proc = subprocess.Popen(
		['python3', SCRIPTDIR + 'battery.py'], stdout=subprocess.PIPE)
	for line in iter(proc.stdout.readline, ''):
		match = re.search("(\d+): (\d+.\d+)", line)
		if match:
			addr = int(match.group(1))
			voltage = match.group(2)[:4] # truncate digits
			color = '#FFFF00' if float(voltage) < 3.7 else defaultBackground
			widgets[addr].config(background=color)
			widgets[addr].config(text="{}\n{}v".format(addr, voltage))

scriptButtons = Tkinter.Frame(top)
batteryButton = Tkinter.Button(scriptButtons, text="battery", command=checkBattery)
batteryButton.pack(side='left')
sysOffButton = Tkinter.Button(scriptButtons, text="sysOff", command=sysOff)
sysOffButton.pack(side='left')
rebootButton = Tkinter.Button(scriptButtons, text="reboot", command=reboot)
rebootButton.pack(side='left')
flashButton = Tkinter.Button(scriptButtons, text="flash", command=flash)
flashButton.pack(side='left')

# start background threads
def checkBatteryLoop():
	while True:
		# rely on GIL
		checkBattery()
		time.sleep(10.0) # seconds
checkBatteryThread = threading.Thread(target=checkBatteryLoop)
checkBatteryThread.daemon = True # so it exits when the main thread exit
checkBatteryThread.start()

# place the widgets in the window and start
buttons.pack()
frame.pack(padx=10, pady=10)
scriptButtons.pack()
top.mainloop()
