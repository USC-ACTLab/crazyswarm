import argparse

import genericJoystick

class Crazyswarm:
    def __init__(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("--sim", help="Run using simulation", action="store_true")
        parser.add_argument("--vis", help="(sim only) Visualization backend [mpl]", choices=['mpl', 'vispy'], default="mpl")
        parser.add_argument("--dt", help="(sim only) dt [0.1s]", type=float, default=0.1)
        args = parser.parse_args()

        if args.sim:
            import crazyflieSim
            self.timeHelper = crazyflieSim.TimeHelper(args.vis, args.dt)
            self.allcfs = crazyflieSim.CrazyflieServer(self.timeHelper)
        else:
            import crazyflie
            self.timeHelper = crazyflie.TimeHelper()
            self.allcfs = crazyflie.CrazyflieServer()
        self.input = genericJoystick.Joystick(self.timeHelper)
