import argparse

import genericJoystick

class Crazyswarm:
    def __init__(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("--sim", help="Run using simulation", action="store_true")
        args = parser.parse_args()

        if args.sim:
            import crazyflieSim
            self.timeHelper = crazyflieSim.TimeHelper()
            self.allcfs = crazyflieSim.CrazyflieServer(self.timeHelper)
        else:
            import crazyflie
            self.timeHelper = crazyflie.TimeHelper()
            self.allcfs = crazyflie.CrazyflieServer()
        self.input = genericJoystick.Joystick(self.timeHelper)
