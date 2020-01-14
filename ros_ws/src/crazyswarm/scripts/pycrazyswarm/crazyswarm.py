import argparse

from . import genericJoystick

# Building the parser in a separate function allows sphinx-argparse to
# auto-generate the documentation for the command-line flags.
def build_argparser():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--sim", help="Run using simulation.", action="store_true")
    parser.add_argument("--vis", help="(sim only) Visualization backend.", choices=['mpl', 'vispy'], default="mpl")
    parser.add_argument("--dt", help="(sim only) Duration of seconds between rendered visualization frames.", type=float, default=0.1)
    parser.add_argument("--writecsv", help="(sim only) Enable CSV output.", action="store_true")
    return parser


class Crazyswarm:
    def __init__(self, launch_yaml=None):
        parser = build_argparser()
        args, unknown = parser.parse_known_args()

        if launch_yaml is None:
            launch_yaml = "../launch/crazyflies.yaml"
        if launch_yaml.endswith(".yaml"):
            launch_yaml = open(launch_yaml, 'r').read()

        if args.sim:
            import crazyflieSim
            self.timeHelper = crazyflieSim.TimeHelper(args.vis, args.dt, args.writecsv)
            self.allcfs = crazyflieSim.CrazyflieServer(self.timeHelper, launch_yaml)
        else:
            import crazyflie
            self.allcfs = crazyflie.CrazyflieServer(launch_yaml)
            self.timeHelper = crazyflie.TimeHelper()
            if args.writecsv:
                print("WARNING: writecsv argument ignored! This is only available in simulation.")
        self.input = genericJoystick.Joystick(self.timeHelper)
