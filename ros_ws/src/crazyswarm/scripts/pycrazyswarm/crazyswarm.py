import argparse

from . import genericJoystick

class Crazyswarm:
    def __init__(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("--sim", help="Run using simulation", action="store_true")
        parser.add_argument("--vis", help="(sim only) Visualization backend [mpl]", choices=['mpl', 'vispy'], default="mpl")
        parser.add_argument("--dt", help="(sim only) dt [0.1s]", type=float, default=0.1)
        parser.add_argument("--writecsv", help="Enable CSV output (only available in simulation)", action="store_true")
        args = parser.parse_args()

        if args.sim:
            import crazyflieSim
            self.timeHelper = crazyflieSim.TimeHelper(args.vis, args.dt, args.writecsv)
            self.allcfs = crazyflieSim.CrazyflieServer(self.timeHelper)
        else:
            import crazyflie
            self.allcfs = crazyflie.CrazyflieServer()
            self.timeHelper = crazyflie.TimeHelper()
            if args.writecsv:
                print("WARNING: writecsv argument ignored! This is only available in simulation.")
        self.input = genericJoystick.Joystick(self.timeHelper)
