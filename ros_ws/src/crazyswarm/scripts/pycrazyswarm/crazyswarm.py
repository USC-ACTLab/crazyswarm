import argparse
import atexit

from . import genericJoystick

# Building the parser in a separate function allows sphinx-argparse to
# auto-generate the documentation for the command-line flags.
def build_argparser(parent_parsers=[]):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        parents=parent_parsers
    )
    parser.add_argument("--sim", help="Run using simulation.", action="store_true")

    group = parser.add_argument_group("Simulation-only", "")
    group.add_argument("--vis", help="Visualization backend.", choices=['mpl', 'vispy', 'vispy_headless', 'null'], default="mpl")
    group.add_argument("--dt", help="Duration of seconds between rendered visualization frames.", type=float, default=0.1)
    group.add_argument("--writecsv", help="Enable CSV output.", action="store_true")
    group.add_argument("--disturbance", help="Simulate Gaussian-distributed disturbance when using cmdVelocityWorld.", type=float, default=0.0)
    group.add_argument("--video", help="Video output path.", type=str)
    return parser


class Crazyswarm:
    def __init__(self, crazyflies_yaml=None, parent_parser=None, args=None):
        if parent_parser is not None:
            parents = [parent_parser]
        else:
            parents = []
        parser = build_argparser(parents)
        if isinstance(args, str):
            args = args.split()
        args, unknown = parser.parse_known_args(args)

        if crazyflies_yaml is None:
            crazyflies_yaml = "../launch/crazyflies.yaml"
        if crazyflies_yaml.endswith(".yaml"):
            crazyflies_yaml = open(crazyflies_yaml, 'r').read()

        if args.sim:
            import crazyflieSim
            self.timeHelper = crazyflieSim.TimeHelper(args.vis, args.dt, args.writecsv, args.disturbance, args.video)
            self.allcfs = crazyflieSim.CrazyflieServer(self.timeHelper, crazyflies_yaml)
            atexit.register(self.timeHelper._atexit)
        else:
            import crazyflie
            self.allcfs = crazyflie.CrazyflieServer(crazyflies_yaml)
            self.timeHelper = crazyflie.TimeHelper()
            if args.writecsv:
                print("WARNING: writecsv argument ignored! This is only available in simulation.")
            if args.video != "":
                print("WARNING: video argument ignored! This is only available in simulation.")
        self.input = genericJoystick.Joystick(self.timeHelper)
