import time
import copy
# import pyglet
from . import keyboard

# class JoyStickHandler:
#     def __init__(self):
#         self.buttonWasPressed = False

#     def on_joybutton_press(joystick, button):
#         if button == 5:
#             self.buttonWasPressed = True

#     def on_joybutton_release(joystick, button):
#         pass

#     def on_joyaxis_motion(joystick, axis, value):
#         pass

#     def on_joyhat_motion(joystick, hat_x, hat_y):
#         pass

class Joystick:
    def __init__(self, timeHelper):
        # joysticks = pyglet.input.get_joysticks()
        # joystick = joysticks[0]
        # joystick.open()
        # self.buttonWasPressed = False
        # joystick.push_handlers(self)
        self.timeHelper = timeHelper
        self.joyID = None

        try:
            from . import linuxjsdev
            self.js = linuxjsdev.Joystick()
            devices = self.js.devices()
            if len(devices) == 0:
                print("Warning: No joystick found!")
            else:
                ids = [dev["id"] for dev in devices]
                # For backwards compatibility, always choose device 0 if available.
                self.joyID = 0 if 0 in ids else devices[0]["id"]
                self.js.open(self.joyID)
        except ImportError:
            print("Warning: Joystick only supported on Linux.")

    # def on_joybutton_press(joystick, button):
        # print(button)
        # if button == 5:
            # self.buttonWasPressed = True

    # def wasButtonPressed(self):
    #     return self.buttonWasPressed

    # def clearButtonPressed(self):
    #     self.buttonWasPressed = False

    def checkIfButtonIsPressed(self):
        if self.joyID is not None:
            state = self.js.read(self.joyID)
            return state[1][5] == 1
        else:
            return False

    def waitUntilButtonPressed(self):
        if self.joyID is not None:
            while not self.checkIfButtonIsPressed():
                self.timeHelper.sleep(0.01)
            while self.checkIfButtonIsPressed():
                self.timeHelper.sleep(0.01)
        else:
            with keyboard.KeyPoller() as keyPoller:
                # Wait until a key is pressed.
                while keyPoller.poll() is None:
                    self.timeHelper.sleep(0.01)
                # Wait until the key is released.
                while keyPoller.poll() is not None:
                    self.timeHelper.sleep(0.01)


    def checkIfAnyButtonIsPressed(self):
        if self.joyID is not None:
            state = self.js.read(self.joyID)
            if state[1][5] == 1 or state[1][4] == 1 or state[1][3] == 1:
                return state[1]
            else:
                return None
        else:
            return None

    def waitUntilAnyButtonPressed(self):
        if self.joyID is not None:
            buttons = self.checkIfAnyButtonIsPressed()
            while buttons is None:
                self.timeHelper.sleep(0.01)
                buttons = copy.copy(self.checkIfAnyButtonIsPressed())
            while self.checkIfAnyButtonIsPressed() is not None:
                self.timeHelper.sleep(0.01)
            return buttons
