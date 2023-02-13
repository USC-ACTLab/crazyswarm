from __future__ import annotations

from rclpy.node import Node
from ..sim_data_types import State, Action

import copy
import numpy as np
import rowan
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

class Visualization:
    """Plots current and desired states into a PDF"""

    def __init__(self, node: Node, params: dict, names: list[str], states: list[State]):
        self.node = node
        self.names = names
        self.ts = []
        self.all_states = []
        self.all_states_desired = []
        self.all_actions = []
        self.filename = params["output_file"]

    def step(self, t, states: list[State], states_desired: list[State], actions: list[Action]):
        self.ts.append(t)
        self.all_states.append(copy.deepcopy(states))
        self.all_states_desired.append(copy.deepcopy(states_desired))
        self.all_actions.append(copy.deepcopy(actions))

    def shutdown(self):

        for k, name in enumerate(self.names):

            cf_states = np.array([s[k]._state for s in self.all_states])
            cf_states_desired = np.array([s[k]._state for s in self.all_states_desired])
            cf_actions = np.array([s[k]._action for s in self.all_actions])

            with PdfPages(self.filename) as pdf:

                # position
                fig, axs = plt.subplots(3, 1, sharex=True)
                axs[0].set_ylabel("px [m]")
                axs[1].set_ylabel("py [m]")
                axs[2].set_ylabel("pz [m]")
                axs[-1].set_xlabel("Time [s]")

                for d in range(3):
                    axs[d].plot(self.ts, cf_states[:,d], label="state")
                    axs[d].plot(self.ts, cf_states_desired[:,d], label="desired")
                axs[0].legend()
                pdf.savefig(fig)
                plt.close()

                # velocity
                fig, axs = plt.subplots(3, 1, sharex=True)
                axs[0].set_ylabel("vx [m/s]")
                axs[1].set_ylabel("vy [m/s]")
                axs[2].set_ylabel("vz [m/s]")
                axs[-1].set_xlabel("Time [s]")

                for d in range(3):
                    axs[d].plot(self.ts, cf_states[:,3+d], label="state")
                    axs[d].plot(self.ts, cf_states_desired[:,3+d], label="desired")
                axs[0].legend()
                pdf.savefig(fig)
                plt.close()

                # orientation
                fig, axs = plt.subplots(3, 1, sharex=True)
                axs[0].set_ylabel("roll [deg]")
                axs[1].set_ylabel("pitch [deg]")
                axs[2].set_ylabel("yaw [deg]")
                axs[-1].set_xlabel("Time [s]")

                rpy = np.degrees(rowan.to_euler(cf_states[:,6:10], convention='xyz'))
                rpy_desired = np.degrees(rowan.to_euler(cf_states_desired[:,6:10], convention='xyz'))

                for d in range(3):
                    axs[d].plot(self.ts, rpy[:,d], label="state")
                    axs[d].plot(self.ts, rpy_desired[:,d], label="desired")
                axs[0].legend()
                pdf.savefig(fig)
                plt.close()

                # omega
                fig, axs = plt.subplots(3, 1, sharex=True)
                axs[0].set_ylabel("wx [deg/s]")
                axs[1].set_ylabel("wy [deg/s]")
                axs[2].set_ylabel("wz [deg/s]")
                axs[-1].set_xlabel("Time [s]")

                for d in range(3):
                    axs[d].plot(self.ts, np.degrees(cf_states[:,10+d]), label="state")
                    axs[d].plot(self.ts, np.degrees(cf_states_desired[:,10+d]), label="desired")
                axs[0].legend()
                pdf.savefig(fig)
                plt.close()

                # actions
                fig, axs = plt.subplots(2, 2, sharex=True, sharey=True)
                axs[0,0].set_ylabel("rpm")
                axs[1,0].set_ylabel("rpm")
                axs[1,0].set_xlabel("Time [s]")
                axs[1,1].set_xlabel("Time [s]")

                axs[0,0].plot(self.ts, cf_actions[:,3], label="M4")
                axs[0,0].set_title("M4")
                axs[0,1].plot(self.ts, cf_actions[:,0], label="M1")
                axs[0,1].set_title("M1")
                axs[1,1].plot(self.ts, cf_actions[:,1], label="M2")
                axs[1,1].set_title("M2")
                axs[1,0].plot(self.ts, cf_actions[:,2], label="M3")
                axs[1,0].set_title("M3")

                pdf.savefig(fig)
                plt.close()

