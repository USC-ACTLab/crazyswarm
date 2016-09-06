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

def save():
	nodes = [node for id, node in all49.items() if widgets[id].checked.get()]
	with open("../launch/crazyflies.yaml", 'w') as outfile:
		yaml.dump({"crazyflies": nodes}, outfile)

all49 = read_by_id("../launch/all49.yaml")
assert(len(all49) == 49)
enabled = read_by_id("../launch/crazyflies.yaml").keys()

# compute absolute pixel coordinates from the initial positions
positions = [node["initialPosition"] for node in all49.values()]
DOWN_DIR = [-1, 0]
RIGHT_DIR = [0, -1]
def dot(a, b):
	return a[0] * b[0] + a[1] * b[1]
pixel_x = [120 * dot(pos, RIGHT_DIR) for pos in positions]
pixel_y = [120 * dot(pos, DOWN_DIR) for pos in positions]
xmin, ymin = min(pixel_x), min(pixel_y)
xmax, ymax = max(pixel_x), max(pixel_y)

# construct the main window
top = Tkinter.Tk()
top.title('Crazyflie Chooser')

# construct the frame containing the absolute-positioned checkboxes
width = xmax - xmin + 50 # account for checkbox + text width
height = ymax - ymin + 30 # account for checkbox + text height
frame = Tkinter.Frame(top, width=width, height=height)

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

# construct all the checkboxes
widgets = {}
for (id, node), x, y in zip(all49.items(), pixel_x, pixel_y):
	w = CFWidget(frame, str(id))
	w.place(x = x - xmin, y = y - ymin)
	w.checked.set(id in enabled)
	widgets[id] = w

# dragging functionality - TODO alt-drag to deselect
drag_start = None
drag_startstate = None

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

top.bind('<ButtonPress-1>', mouseDown)
top.bind('<ButtonPress-3>', mouseDown)
top.bind('<B1-Motion>', lambda event: drag(event, True))
top.bind('<B3-Motion>', lambda event: drag(event, False))
top.bind('<ButtonRelease-1>', mouseUp)
top.bind('<ButtonRelease-3>', mouseUp)

# buttons for clearing/filling all checkboxes
def clear():
	for box in widgets.values():
		box.checked.set(False)
	save()

def fill():
	for box in widgets.values():
		box.checked.set(True)
	save()

def mkbutton(parent, name, command):
	button = Tkinter.Button(parent, text=name, command=command)
	button.pack(side='left')

buttons = Tkinter.Frame(top)
mkbutton(buttons, "Clear", clear)
mkbutton(buttons, "Fill", fill)

# construct bottom buttons for utility scripts
def sysOff():
	subprocess.Popen(["python3", SCRIPTDIR + "sysOff.py"])
def reboot():
	subprocess.Popen(["python3", SCRIPTDIR + "rebootAll.py"])
def flash():
	subprocess.Popen(["python3", SCRIPTDIR + "flashAll.py", "-stm32"])

import random
def checkBattery():
	for id, w in widgets.items():
		w.batteryLabel.config(foreground='#999999')
	proc = subprocess.Popen(
		['python3', SCRIPTDIR + 'battery.py'], stdout=subprocess.PIPE)
	#for id, widget in widgets.items():
		#line = "{}: {}".format(id, 3.3 + random.random())
		#if not widget.checked.get():
			#line = ""
	for line in iter(proc.stdout.readline, ''):
		match = re.search("(\d+): (\d+.\d+)", line)
		if match:
			addr = int(match.group(1))
			voltage = match.group(2)[:4] # truncate digits
			color = '#000000'
			if float(voltage) < 3.8: color = '#FF8800'
			if float(voltage) < 3.7: color = '#FF0000'
			widgets[addr].batteryLabel.config(foreground=color, text=voltage + ' v')

scriptButtons = Tkinter.Frame(top)
mkbutton(scriptButtons, "battery", checkBattery)
mkbutton(scriptButtons, "sysOff", sysOff)
mkbutton(scriptButtons, "reboot", reboot)
mkbutton(scriptButtons, "flash", flash)

# start background threads
def checkBatteryLoop():
	while True:
		# rely on GIL
		checkBattery()
		time.sleep(10.0) # seconds
# checkBatteryThread = threading.Thread(target=checkBatteryLoop)
# checkBatteryThread.daemon = True # so it exits when the main thread exit
# checkBatteryThread.start()

# place the widgets in the window and start
buttons.pack()
frame.pack(padx=10, pady=10)
scriptButtons.pack()
top.mainloop()
