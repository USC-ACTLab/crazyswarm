import time
# import pyglet
import linuxjsdev

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
        state = self.js.read(0)
        return state[1][5] == 1

    def waitUntilButtonPressed(self):
        while not self.checkIfButtonIsPressed():
            self.timeHelper.sleep(0.01)
        while self.checkIfButtonIsPressed():
            self.timeHelper.sleep(0.01)
