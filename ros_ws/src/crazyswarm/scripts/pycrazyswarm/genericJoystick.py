import time
# import pyglet
from . import linuxjsdev
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

        self.js = linuxjsdev.Joystick()
        dummy = self.js.devices()
        if len(dummy) == 0:
            print("Warning: No joystick found!")
            self.hasJoystick = False
        else:
            self.hasJoystick = True
            self.js.open(0)

    # def on_joybutton_press(joystick, button):
        # print(button)
        # if button == 5:
            # self.buttonWasPressed = True

    # def wasButtonPressed(self):
    #     return self.buttonWasPressed

    # def clearButtonPressed(self):
    #     self.buttonWasPressed = False

    def checkIfButtonIsPressed(self):
        if self.hasJoystick:
            state = self.js.read(0)
            return state[1][5] == 1
        else:
            return False

    def waitUntilButtonPressed(self):
        if self.hasJoystick:
            while not self.checkIfButtonIsPressed():
                self.timeHelper.sleep(0.01)
            while self.checkIfButtonIsPressed():
                self.timeHelper.sleep(0.01)
        else:
            with keyboard.KeyPoller() as keyPoller:
                while keyPoller.poll() is None:
                    self.timeHelper.sleep(0.01)
                while keyPoller.poll() is not None:
                    self.timeHelper.sleep(0.01)
        self.timeHelper.nextPhase()
