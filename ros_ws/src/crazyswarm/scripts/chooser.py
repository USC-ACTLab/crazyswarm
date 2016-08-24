import Tkinter
import yaml
import os
import subprocess

os.chdir(os.path.dirname(os.path.realpath(__file__)))
SCRIPTDIR = "../../../../scripts/"

# read the yaml files
def read_by_id(path):
	by_id = {}
	with open(path, 'r') as ymlfile:
		root = yaml.load(ymlfile)
		for node in root["crazyflies"]:
			id = int(node["id"])
			by_id[id] = node
	return by_id
	

all49 = read_by_id("../launch/all49.yaml")
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
width = int(xmax - xmin + 70)
height = int(ymax - ymin + 150) # extra room for buttons at top and bottom
top.geometry("{0}x{1}".format(str(width), str(height)))

# construct all the checkboxes
toggles = {} # map crazyflie id to Tkinter data-binding variable
widgets = []
for (id, node), x, y in zip(all49.items(), pixel_x, pixel_y):
	toggles[id] = Tkinter.BooleanVar()
	toggles[id].set(id in enabled)
	checkbox = Tkinter.Checkbutton(top, text=str(id).zfill(2), 
		justify='left', variable=toggles[id])
	checkbox.place(x = x - xmin + 10, y = y - ymin + 50)
	widgets.append(checkbox)

# dragging - TODO alt-drag to deselect
drag_startx = None
drag_starty = None
drag_startstate = None

def mouseDown(event):
	global drag_startx, drag_starty, drag_startstate
	drag_starty = event.y
	drag_startx = event.x
	drag_startstate = [toggle.get() for toggle in toggles.values()]

def drag(event):
	dragx0 = min(drag_startx, event.x)
	dragx1 = max(drag_startx, event.x)
	dragy0 = min(drag_starty, event.y)
	dragy1 = max(drag_starty, event.y)
	def dragcontains(widget):
		x0 = widget.winfo_x()
		y0 = widget.winfo_y()
		x1 = x0 + widget.winfo_width()
		y1 = y0 + widget.winfo_height()
		return not (
			x0 > dragx1 or
			x1 < dragx0 or
			y0 > dragy1 or
			y1 < dragy0)
		
	for initial, toggle, checkbox in zip(drag_startstate, toggles.values(), widgets):
		if dragcontains(checkbox):
			toggle.set(True)
		else:
			toggle.set(initial)

top.bind('<ButtonPress-1>', mouseDown) 
top.bind('<B1-Motion>', drag) 

# construct top buttons
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
buttons.pack()
saveButton = Tkinter.Button(buttons, text="Save", command=save)
saveButton.pack(side='left')
clearButton = Tkinter.Button(buttons, text="Clear", command=clear)
clearButton.pack(side='left')
fillButton = Tkinter.Button(buttons, text="Fill", command=fill)
fillButton.pack(side='left')

#construct bottom (script) buttons
def sysOff():
	subprocess.Popen(["python3", SCRIPTDIR + "sysOff.py"])
def reboot():
	subprocess.Popen(["python3", SCRIPTDIR + "rebootAll.py"])
def flash():
	subprocess.Popen(["python3", SCRIPTDIR + "flashAll.py", "-stm32"])
	
scriptButtons = Tkinter.Frame(top)
scriptButtons.pack(side='bottom')
sysOffButton = Tkinter.Button(scriptButtons, text="sysOff", command=sysOff)
sysOffButton.pack(side='left')
rebootButton = Tkinter.Button(scriptButtons, text="reboot", command=reboot)
rebootButton.pack(side='left')
flashButton = Tkinter.Button(scriptButtons, text="flash", command=flash)
flashButton.pack(side='left')


# run gui
top.mainloop()
