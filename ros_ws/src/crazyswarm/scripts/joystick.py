import time
import rospy
from sensor_msgs.msg import Joy

class Joystick:
    def __init__(self):
        self.lastButtonState = 0
        self.buttonWasPressed = False
        rospy.Subscriber("/joy", Joy, joyChanged, self)

    def joyChanged(self, data):
        if (not self.buttonWasPressed and
            data.buttons[5] == 1 and
            self.lastButtonState == 0):
            self.buttonWasPressed = True
        self.lastButtonState = data.buttons[5]

    def waitUntilButtonPressed():
        while not rospy.is_shutdown() and not self.buttonWasPressed:
            time.sleep(0.01)
        self.buttonWasPressed = False
